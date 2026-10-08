`timescale 1ns / 1ps

// ================================================================
// M18.4 cache.v -- shared 256/512/1024-set, 2-way/4-way ICache/DCache
// with the functionally-correct M17.3F HUR replay fix
// post-critical hit-under-refill support
//
// Geometry:
//   - 16 bytes per line, write-back + write-allocate
//   - SETS_512 = 0: 256 sets, index addr[11:4], tag paddr[31:12]
//   - SETS_512 = 1: 512 sets, index addr[12:4], tag paddr[31:13]
//   - SETS_1024 = 1: 1024 sets, index addr[13:4], tag paddr[31:14]
//   - FOUR_WAY = 0/1 selects 2-way/4-way associativity
//   - M17.5D default DCache: 512 sets x 4 ways = 32KB
//   - M17.4I ICache remains 256 sets x 4 ways = 16KB
//
// 256-set mode keeps the original Xilinx IPs:
//   tagv_sram       : 256 x 21
//   data_bank_sram  : 256 x 32 with byte write enable
// 512-set mode uses the inferred block-RAM modules embedded at the end of
// this file.  Each 512 x 32 data bank and 512 x 21 tag RAM fits one RAMB18.
// ================================================================
module cache #(
    parameter PIPELINED_HIT    = 1'b0,
    parameter FOUR_WAY         = 1'b0,
    parameter SETS_512         = 1'b0,
    parameter SETS_1024        = 1'b0,
    // DCache-only option.  The cache still permits only one outstanding
    // memory-line miss, but after the critical word has been returned it may
    // accept one younger cached load.  Same-line words are forwarded from the
    // refill buffer; hits to other resident lines use the otherwise-idle RAM
    // read port.  A second miss is replayed after the active refill completes.
    parameter HIT_UNDER_REFILL = 1'b0,
    // Bandwidth option. A demand miss reads at least the requested 16-byte line
    // plus the following two lines in one AXI burst. The demand line commits
    // after beat 3 and speculative sectors install only at 16-byte boundaries.
    parameter ADJACENT_LINE_FILL = 1'b0,
    // Legal AXI3 maximum: four 16-byte sectors in one 16-beat burst.  Used by
    // ICache, whose mostly sequential stream benefits from the fourth sector.
    parameter FOUR_LINE_FILL = 1'b0,
    // M18.3 extends the 12-beat transaction to 20 beats, adding the +48 and
    // +64 byte sectors.  This reduces both contiguous and 32-byte-stride demand
    // misses while still paying the competition RAM's long AR delay only once.
    parameter FIVE_LINE_FILL = 1'b0,
    // ICache sets this because every resident line is clean.  A speculative
    // sector may therefore use the normal round-robin victim when the set is
    // full.  DCache leaves it clear and only consumes invalid ways.
    parameter READ_ONLY_CACHE = 1'b0
)(
    input  wire        clk,
    input  wire        resetn,

    // CPU side
    input  wire        valid,
    input  wire        op,        // 0: read, 1: write
    input  wire [ 9:0] index,
    input  wire [19:0] tag,
    input  wire [ 3:0] offset,
    input  wire [ 3:0] wstrb,
    input  wire [31:0] wdata,
    // Instruction-side same-line dual-word read. DCache ties this low.
    input  wire        dual_read,

    // CACOP sideband. For indexed operations, cacop_way selects the way:
    // 2-way cache uses cacop_way[0]; 4-way cache uses cacop_way[1:0].
    input  wire        cacop_valid,
    input  wire [ 4:0] cacop_code,
    input  wire [ 9:0] cacop_index,
    input  wire [19:0] cacop_tag,
    input  wire [ 1:0] cacop_way,
    output wire        cacop_addr_ok,
    output reg         cacop_data_ok,

    output wire        addr_ok,
    output reg         data_ok,
    output reg  [31:0] rdata,
    output reg  [31:0] rdata2,

    // memory / AXI-bridge side
    output wire        rd_req,
    output wire [ 2:0] rd_type,
    output wire [31:0] rd_addr,
    input  wire        rd_rdy,
    input  wire        ret_valid,
    input  wire        ret_last,
    input  wire [31:0] ret_data,

    output wire        wr_req,
    output wire [ 2:0] wr_type,
    output wire [31:0] wr_addr,
    output wire [ 3:0] wr_wstrb,
    output wire [127:0] wr_data,
    input  wire        wr_rdy
);

// ------------------------------------------------------------
// Parameters and state
// ------------------------------------------------------------
localparam S_INIT      = 3'd0;
localparam S_IDLE      = 3'd1;
localparam S_LOOKUP    = 3'd2;
localparam S_WB        = 3'd3;
localparam S_RDREQ     = 3'd4;
localparam S_REFILL    = 3'd5;
localparam S_CACOP     = 3'd6;
localparam S_CACOP_WB  = 3'd7;
localparam TYPE_LINE   = 3'b100;
localparam TYPE_FOUR_LINE = 3'b101;
localparam TYPE_TRIPLE_LINE = 3'b110;
localparam TYPE_FIVE_LINE = 3'b111;
localparam [9:0] LAST_INDEX = SETS_1024 ? 10'h3ff :
                              SETS_512  ? 10'h1ff : 10'h0ff;

reg [2:0] state;
reg [9:0] init_index;

// M18.4T stores dirty state in asynchronous-read distributed RAM instead of
// 4096 independently reset flops.  S_INIT clears one set per cycle alongside
// tag initialization, so no bulk reset or 1024:1 flop mux remains on lookup.
(* ram_style = "distributed" *) reg dirty_way0 [0:1023];
(* ram_style = "distributed" *) reg dirty_way1 [0:1023];
(* ram_style = "distributed" *) reg dirty_way2 [0:1023];
(* ram_style = "distributed" *) reg dirty_way3 [0:1023];

// Request buffer
reg        req_op;
reg [ 9:0] req_index;
reg [19:0] req_tag;
reg [ 3:0] req_offset;
reg [ 3:0] req_wstrb;
reg [31:0] req_wdata;
reg        req_dual_read;
reg [ 4:0] req_cacop_code;
reg [ 1:0] req_cacop_way;

// Miss / replacement buffer
reg [ 1:0] repl_way;
reg [19:0] repl_old_tag;
reg [127:0] repl_old_data;
reg [127:0] refill_buf;
reg [127:0] adjacent_refill_buf;
reg [127:0] stride_refill_buf;
reg [127:0] third_refill_buf;
reg [127:0] fourth_refill_buf;
reg [ 4:0] ret_cnt;
reg [ 1:0] random_way;
reg         miss_resp_sent;

// M17.3F single-MSHR / hit-under-refill request buffer.  Only younger loads
// are accepted while a read miss is finishing; stores and a second miss remain
// ordered behind the active refill.
reg         hur_valid;
reg         hur_probe_pending;
reg         hur_second_miss;
reg         hur_replay_setup;
reg         hur_replay_active;
reg [ 9:0]  hur_index;
reg [19:0]  hur_tag;
reg [ 3:0]  hur_offset;
reg [ 3:0]  refill_valid_mask;

`ifndef SYNTHESIS
reg [31:0] hur_accept_cnt;
reg [31:0] hur_same_line_hit_cnt;
reg [31:0] hur_other_line_hit_cnt;
reg [31:0] hur_second_miss_cnt;
reg [31:0] hur_word_wait_cycle_cnt;
reg [31:0] hur_store_block_cycle_cnt;
reg [31:0] hur_replay_setup_cnt;
reg [31:0] hur_replay_hit_cnt;
reg [31:0] hur_replay_miss_cnt;
reg [31:0] hur_debug_cycle_cnt;
reg [31:0] hur_debug_event_seq;
// Generic cache counters are simulation-only and work for either ICache or
// DCache.  M17.4I uses the ICache instance to measure whether 4-way capacity
// actually removes frontend misses; synthesis remains unaffected.
reg [31:0] sim_lookup_cnt;
reg [31:0] sim_hit_cnt;
reg [31:0] sim_miss_cnt;
reg [31:0] sim_refill_line_cnt;
reg [31:0] sim_rdreq_cycle_cnt;
reg [31:0] sim_refill_cycle_cnt;
reg [31:0] sim_way0_hit_cnt;
reg [31:0] sim_way1_hit_cnt;
reg [31:0] sim_way2_hit_cnt;
reg [31:0] sim_way3_hit_cnt;
reg [31:0] sim_way0_victim_cnt;
reg [31:0] sim_way1_victim_cnt;
reg [31:0] sim_way2_victim_cnt;
reg [31:0] sim_way3_victim_cnt;
`endif

// ------------------------------------------------------------
// Helper functions
// ------------------------------------------------------------
function [31:0] select_word;
    input [127:0] line;
    input [1:0]   word_off;
    begin
        case (word_off)
            2'd0: select_word = line[ 31:  0];
            2'd1: select_word = line[ 63: 32];
            2'd2: select_word = line[ 95: 64];
            default: select_word = line[127: 96];
        endcase
    end
endfunction

function [31:0] merge_word;
    input [31:0] old_word;
    input [31:0] new_word;
    input [3:0]  byte_en;
    begin
        merge_word[ 7: 0] = byte_en[0] ? new_word[ 7: 0] : old_word[ 7: 0];
        merge_word[15: 8] = byte_en[1] ? new_word[15: 8] : old_word[15: 8];
        merge_word[23:16] = byte_en[2] ? new_word[23:16] : old_word[23:16];
        merge_word[31:24] = byte_en[3] ? new_word[31:24] : old_word[31:24];
    end
endfunction

function [127:0] insert_word;
    input [127:0] line;
    input [1:0]   word_off;
    input [31:0]  word_data;
    begin
        insert_word = line;
        case (word_off)
            2'd0: insert_word[ 31:  0] = word_data;
            2'd1: insert_word[ 63: 32] = word_data;
            2'd2: insert_word[ 95: 64] = word_data;
            default: insert_word[127: 96] = word_data;
        endcase
    end
endfunction

function [31:0] compose_line_addr;
    input [19:0] line_tag;
    input [ 9:0] line_index;
    begin
        if (SETS_1024)
            compose_line_addr = {line_tag[17:0], line_index, 4'b0000};
        else if (SETS_512)
            compose_line_addr = {line_tag[18:0], line_index[8:0], 4'b0000};
        else
            compose_line_addr = {line_tag, line_index[7:0], 4'b0000};
    end
endfunction

function [31:0] compose_byte_addr;
    input [19:0] line_tag;
    input [ 9:0] line_index;
    input [ 3:0] byte_offset;
    begin
        compose_byte_addr = compose_line_addr(line_tag, line_index) |
                            {28'b0, byte_offset};
    end
endfunction

// ------------------------------------------------------------
// RAM address selection
// ------------------------------------------------------------
wire       cacop_accept  = (state == S_IDLE) && cacop_valid;
wire       lookup_accept = (state == S_IDLE) && !cacop_valid && valid;
wire       lookup_hit_accept;
wire       refill_last;
wire       adjacent_install;
wire [1:0] adjacent_choose_way;
reg        adjacent_install_armed;
reg  [1:0] adjacent_choose_way_r;

// Generate the following line addresses in cache-line units.  The previous
// implementation reconstructed a 32-bit byte address and then added
// 16/32/48/64.  That inferred a five-CARRY4 chain from req_index into the
// speculative-install controls and, in the official 100 MHz route, became the
// worst DCache-to-BRAM setup path.  Only the active set-index bits participate
// in this operation.  Split the index increment from the one-bit tag carry so
// the common non-wrapping path remains narrow while preserving exact physical
// address wrap behaviour for every supported geometry.
wire [10:0] adjacent_sum_1024 = {1'b0, req_index[9:0]} + 11'd1;
wire [10:0] stride_sum_1024   = {1'b0, req_index[9:0]} + 11'd2;
wire [10:0] third_sum_1024    = {1'b0, req_index[9:0]} + 11'd3;
wire [10:0] fourth_sum_1024   = {1'b0, req_index[9:0]} + 11'd4;
wire [ 9:0] adjacent_sum_512  = {1'b0, req_index[8:0]} + 10'd1;
wire [ 9:0] stride_sum_512    = {1'b0, req_index[8:0]} + 10'd2;
wire [ 9:0] third_sum_512     = {1'b0, req_index[8:0]} + 10'd3;
wire [ 9:0] fourth_sum_512    = {1'b0, req_index[8:0]} + 10'd4;
wire [ 8:0] adjacent_sum_256  = {1'b0, req_index[7:0]} + 9'd1;
wire [ 8:0] stride_sum_256    = {1'b0, req_index[7:0]} + 9'd2;
wire [ 8:0] third_sum_256     = {1'b0, req_index[7:0]} + 9'd3;
wire [ 8:0] fourth_sum_256    = {1'b0, req_index[7:0]} + 9'd4;

wire [ 9:0] adjacent_index = SETS_1024 ? adjacent_sum_1024[9:0] :
                                      SETS_512 ? {1'b0, adjacent_sum_512[8:0]} :
                                                 {2'b0, adjacent_sum_256[7:0]};
wire [ 9:0] stride_index = SETS_1024 ? stride_sum_1024[9:0] :
                                    SETS_512 ? {1'b0, stride_sum_512[8:0]} :
                                               {2'b0, stride_sum_256[7:0]};
wire [ 9:0] third_index = SETS_1024 ? third_sum_1024[9:0] :
                                   SETS_512 ? {1'b0, third_sum_512[8:0]} :
                                              {2'b0, third_sum_256[7:0]};
wire [ 9:0] fourth_index = SETS_1024 ? fourth_sum_1024[9:0] :
                                    SETS_512 ? {1'b0, fourth_sum_512[8:0]} :
                                               {2'b0, fourth_sum_256[7:0]};

wire adjacent_wrap = SETS_1024 ? adjacent_sum_1024[10] :
                     SETS_512  ? adjacent_sum_512[9] :
                                 adjacent_sum_256[8];
wire stride_wrap = SETS_1024 ? stride_sum_1024[10] :
                   SETS_512  ? stride_sum_512[9] :
                               stride_sum_256[8];
wire third_wrap = SETS_1024 ? third_sum_1024[10] :
                  SETS_512  ? third_sum_512[9] :
                              third_sum_256[8];
wire fourth_wrap = SETS_1024 ? fourth_sum_1024[10] :
                   SETS_512  ? fourth_sum_512[9] :
                               fourth_sum_256[8];

wire [19:0] adjacent_tag = SETS_1024 ?
                           {2'b0, req_tag[17:0] + {{17{1'b0}}, adjacent_wrap}} :
                           SETS_512 ?
                           {1'b0, req_tag[18:0] + {{18{1'b0}}, adjacent_wrap}} :
                           req_tag + {{19{1'b0}}, adjacent_wrap};
wire [19:0] stride_tag = SETS_1024 ?
                         {2'b0, req_tag[17:0] + {{17{1'b0}}, stride_wrap}} :
                         SETS_512 ?
                         {1'b0, req_tag[18:0] + {{18{1'b0}}, stride_wrap}} :
                         req_tag + {{19{1'b0}}, stride_wrap};
wire [19:0] third_tag = SETS_1024 ?
                        {2'b0, req_tag[17:0] + {{17{1'b0}}, third_wrap}} :
                        SETS_512 ?
                        {1'b0, req_tag[18:0] + {{18{1'b0}}, third_wrap}} :
                        req_tag + {{19{1'b0}}, third_wrap};
wire [19:0] fourth_tag = SETS_1024 ?
                         {2'b0, req_tag[17:0] + {{17{1'b0}}, fourth_wrap}} :
                         SETS_512 ?
                         {1'b0, req_tag[18:0] + {{18{1'b0}}, fourth_wrap}} :
                         req_tag + {{19{1'b0}}, fourth_wrap};
wire [2:0] refill_sector = ret_cnt[4:2];
wire demand_refill_phase = (refill_sector == 3'd0);
wire adjacent_refill_phase = (refill_sector == 3'd1);
wire stride_refill_phase = (refill_sector == 3'd2);
wire third_refill_phase = (refill_sector == 3'd3);
wire fourth_refill_phase = (refill_sector == 3'd4);
wire [9:0] prefetch_index = fourth_refill_phase ? fourth_index :
                            third_refill_phase  ? third_index  :
                            stride_refill_phase ? stride_index : adjacent_index;
wire [19:0] prefetch_tag = fourth_refill_phase ? fourth_tag :
                           third_refill_phase  ? third_tag  :
                           stride_refill_phase ? stride_tag : adjacent_tag;
wire adjacent_probe_phase = ADJACENT_LINE_FILL && (state == S_REFILL) &&
                            !demand_refill_phase;

// A younger load can be accepted only after the active read miss has already
// produced its architectural response.  Do not accept on the final return beat
// because that edge is used to install the completed line.
wire hur_addr_window = HIT_UNDER_REFILL && !ADJACENT_LINE_FILL &&
                       (state == S_REFILL) &&
                       miss_resp_sent && !hur_valid && !cacop_valid && !op &&
                       !(ret_valid && ret_last);
wire hur_accept = hur_addr_window && valid;

// During refill, data/tag RAMs are no longer written beat-by-beat in HUR mode;
// the complete line is committed on ret_last.  This leaves the single RAM port
// available for one younger lookup between return beats.
wire [9:0] ram_addr = (state == S_INIT) ? init_index :
                       cacop_accept      ? cacop_index :
                       (lookup_accept || lookup_hit_accept) ? index :
                       (HIT_UNDER_REFILL && refill_last) ? req_index :
                       adjacent_probe_phase ? prefetch_index :
                       hur_accept ? index :
                      (HIT_UNDER_REFILL && state == S_REFILL && hur_valid) ? hur_index :
                      req_index;

// ------------------------------------------------------------
// TAGV RAMs
// ------------------------------------------------------------
wire [20:0] tagv_way0_rdata;
wire [20:0] tagv_way1_rdata;
wire [20:0] tagv_way2_rdata;
wire [20:0] tagv_way3_rdata;

wire        way0_valid = tagv_way0_rdata[0];
wire        way1_valid = tagv_way1_rdata[0];
wire        way2_valid = FOUR_WAY && tagv_way2_rdata[0];
wire        way3_valid = FOUR_WAY && tagv_way3_rdata[0];
wire [19:0] way0_tag   = tagv_way0_rdata[20:1];
wire [19:0] way1_tag   = tagv_way1_rdata[20:1];
wire [19:0] way2_tag   = tagv_way2_rdata[20:1];
wire [19:0] way3_tag   = tagv_way3_rdata[20:1];
wire        way0_hit   = way0_valid && (way0_tag == req_tag);
wire        way1_hit   = way1_valid && (way1_tag == req_tag);
wire        way2_hit   = way2_valid && (way2_tag == req_tag);
wire        way3_hit   = way3_valid && (way3_tag == req_tag);
wire        cache_hit  = way0_hit || way1_hit || way2_hit || way3_hit;
wire [1:0]  hit_way    = way0_hit ? 2'd0 :
                         way1_hit ? 2'd1 :
                         way2_hit ? 2'd2 : 2'd3;

wire hur_way0_hit = way0_valid && (way0_tag == hur_tag);
wire hur_way1_hit = way1_valid && (way1_tag == hur_tag);
wire hur_way2_hit = way2_valid && (way2_tag == hur_tag);
wire hur_way3_hit = way3_valid && (way3_tag == hur_tag);
wire hur_cache_hit = hur_way0_hit || hur_way1_hit || hur_way2_hit || hur_way3_hit;
wire [1:0] hur_hit_way = hur_way0_hit ? 2'd0 :
                         hur_way1_hit ? 2'd1 :
                         hur_way2_hit ? 2'd2 : 2'd3;
wire hur_same_line = hur_valid && (hur_index == req_index) && (hur_tag == req_tag);

// A load hit leaves the single-port data RAMs free to launch the next lookup.
// A store hit writes the current request's set on this edge, so it must return
// to IDLE instead of selecting the younger request's set as ram_addr.
assign lookup_hit_accept = PIPELINED_HIT && (state == S_LOOKUP) &&
                           cache_hit && !req_op && !cacop_valid && valid;

// CACOP operation decoding
wire [1:0] cacop_mode       = req_cacop_code[4:3];
wire       cacop_hit_mode   = (cacop_mode == 2'b10);
wire       cacop_direct     = !cacop_hit_mode;
wire       cacop_has_target = cacop_direct || cache_hit;
wire [1:0] cacop_index_way  = FOUR_WAY ? req_cacop_way : {1'b0, req_cacop_way[0]};
wire [1:0] cacop_target_way = cacop_hit_mode ? hit_way : cacop_index_way;
wire       cacop_store_tag  = (cacop_mode == 2'b00);

wire cacop_tgt_valid = (cacop_target_way == 2'd0) ? way0_valid :
                       (cacop_target_way == 2'd1) ? way1_valid :
                       (cacop_target_way == 2'd2) ? way2_valid : way3_valid;
wire cacop_tgt_dirty = (cacop_target_way == 2'd0) ? dirty_way0[req_index] :
                       (cacop_target_way == 2'd1) ? dirty_way1[req_index] :
                       (cacop_target_way == 2'd2) ? dirty_way2[req_index] : dirty_way3[req_index];
wire [19:0] cacop_tgt_tag = (cacop_target_way == 2'd0) ? way0_tag :
                            (cacop_target_way == 2'd1) ? way1_tag :
                            (cacop_target_way == 2'd2) ? way2_tag : way3_tag;
wire cacop_need_wb = (state == S_CACOP) && cacop_has_target &&
                     (cacop_mode != 2'b00) && cacop_tgt_valid && cacop_tgt_dirty;
wire cacop_clear_now = (state == S_CACOP) && cacop_has_target && !cacop_need_wb;
wire cacop_wb_done   = (state == S_CACOP_WB) && wr_rdy;

// StoreTag with zero CTAG invalidates every way in the indexed set. Other
// CACOP modes invalidate only the selected / hit way.
wire cacop_clear_way0 = (cacop_clear_now && (cacop_store_tag || cacop_target_way == 2'd0)) ||
                        (cacop_wb_done && repl_way == 2'd0);
wire cacop_clear_way1 = (cacop_clear_now && (cacop_store_tag || cacop_target_way == 2'd1)) ||
                        (cacop_wb_done && repl_way == 2'd1);
wire cacop_clear_way2 = FOUR_WAY &&
                       ((cacop_clear_now && (cacop_store_tag || cacop_target_way == 2'd2)) ||
                        (cacop_wb_done && repl_way == 2'd2));
wire cacop_clear_way3 = FOUR_WAY &&
                       ((cacop_clear_now && (cacop_store_tag || cacop_target_way == 2'd3)) ||
                        (cacop_wb_done && repl_way == 2'd3));

// In double-line mode the architectural demand line is complete at beat 3,
// four cycles before AXI RLAST.  Keep the legacy name for the demand-line
// commit pulse so all existing refill machinery retains its meaning.
assign refill_last = (state == S_REFILL) && ret_valid &&
                     (ADJACENT_LINE_FILL ? (ret_cnt == 5'd3) : ret_last);
wire tagv_init   = (state == S_INIT);
wire tagv_we0    = tagv_init || cacop_clear_way0 ||
                   (refill_last && repl_way == 2'd0) ||
                   (adjacent_install && adjacent_choose_way_r == 2'd0);
wire tagv_we1    = tagv_init || cacop_clear_way1 ||
                   (refill_last && repl_way == 2'd1) ||
                   (adjacent_install && adjacent_choose_way_r == 2'd1);
wire tagv_we2    = tagv_init || cacop_clear_way2 ||
                   (refill_last && repl_way == 2'd2) ||
                   (adjacent_install && adjacent_choose_way_r == 2'd2);
wire tagv_we3    = tagv_init || cacop_clear_way3 ||
                   (refill_last && repl_way == 2'd3) ||
                   (adjacent_install && adjacent_choose_way_r == 2'd3);
wire [20:0] tagv_din0 = (tagv_init || cacop_clear_way0) ? 21'b0 :
                         adjacent_install ? {prefetch_tag, 1'b1} : {req_tag, 1'b1};
wire [20:0] tagv_din1 = (tagv_init || cacop_clear_way1) ? 21'b0 :
                         adjacent_install ? {prefetch_tag, 1'b1} : {req_tag, 1'b1};
wire [20:0] tagv_din2 = (tagv_init || cacop_clear_way2) ? 21'b0 :
                         adjacent_install ? {prefetch_tag, 1'b1} : {req_tag, 1'b1};
wire [20:0] tagv_din3 = (tagv_init || cacop_clear_way3) ? 21'b0 :
                         adjacent_install ? {prefetch_tag, 1'b1} : {req_tag, 1'b1};

generate
    if (SETS_1024) begin : gen_1024_tagv
        m184_tagv_sram_1024 u_tagv_way0 (
            .clka(clk), .ena(1'b1), .wea({tagv_we0}), .addra(ram_addr),
            .dina(tagv_din0), .douta(tagv_way0_rdata)
        );
        m184_tagv_sram_1024 u_tagv_way1 (
            .clka(clk), .ena(1'b1), .wea({tagv_we1}), .addra(ram_addr),
            .dina(tagv_din1), .douta(tagv_way1_rdata)
        );
        if (FOUR_WAY) begin : gen_four_way_tagv_1024
            m184_tagv_sram_1024 u_tagv_way2 (
                .clka(clk), .ena(1'b1), .wea({tagv_we2}), .addra(ram_addr),
                .dina(tagv_din2), .douta(tagv_way2_rdata)
            );
            m184_tagv_sram_1024 u_tagv_way3 (
                .clka(clk), .ena(1'b1), .wea({tagv_we3}), .addra(ram_addr),
                .dina(tagv_din3), .douta(tagv_way3_rdata)
            );
        end
        else begin : gen_two_way_tagv_1024
            assign tagv_way2_rdata = 21'b0;
            assign tagv_way3_rdata = 21'b0;
        end
    end
    else if (SETS_512) begin : gen_512_tagv
        m175d_tagv_sram_512 u_tagv_way0 (
            .clka(clk), .ena(1'b1), .wea({tagv_we0}), .addra(ram_addr[8:0]),
            .dina(tagv_din0), .douta(tagv_way0_rdata)
        );
        m175d_tagv_sram_512 u_tagv_way1 (
            .clka(clk), .ena(1'b1), .wea({tagv_we1}), .addra(ram_addr[8:0]),
            .dina(tagv_din1), .douta(tagv_way1_rdata)
        );
        if (FOUR_WAY) begin : gen_four_way_tagv_512
            m175d_tagv_sram_512 u_tagv_way2 (
                .clka(clk), .ena(1'b1), .wea({tagv_we2}), .addra(ram_addr[8:0]),
                .dina(tagv_din2), .douta(tagv_way2_rdata)
            );
            m175d_tagv_sram_512 u_tagv_way3 (
                .clka(clk), .ena(1'b1), .wea({tagv_we3}), .addra(ram_addr[8:0]),
                .dina(tagv_din3), .douta(tagv_way3_rdata)
            );
        end
        else begin : gen_two_way_tagv_512
            assign tagv_way2_rdata = 21'b0;
            assign tagv_way3_rdata = 21'b0;
        end
    end
    else begin : gen_256_tagv
        tagv_sram u_tagv_way0 (
            .clka(clk), .ena(1'b1), .wea({tagv_we0}), .addra(ram_addr[7:0]),
            .dina(tagv_din0), .douta(tagv_way0_rdata)
        );
        tagv_sram u_tagv_way1 (
            .clka(clk), .ena(1'b1), .wea({tagv_we1}), .addra(ram_addr[7:0]),
            .dina(tagv_din1), .douta(tagv_way1_rdata)
        );
        if (FOUR_WAY) begin : gen_four_way_tagv
            tagv_sram u_tagv_way2 (
                .clka(clk), .ena(1'b1), .wea({tagv_we2}), .addra(ram_addr[7:0]),
                .dina(tagv_din2), .douta(tagv_way2_rdata)
            );
            tagv_sram u_tagv_way3 (
                .clka(clk), .ena(1'b1), .wea({tagv_we3}), .addra(ram_addr[7:0]),
                .dina(tagv_din3), .douta(tagv_way3_rdata)
            );
        end
        else begin : gen_two_way_tagv
            assign tagv_way2_rdata = 21'b0;
            assign tagv_way3_rdata = 21'b0;
        end
    end
endgenerate

// ------------------------------------------------------------
// DATA RAMs: up to 4 ways x 4 banks
// ------------------------------------------------------------
wire [31:0] way0_b0_rdata, way0_b1_rdata, way0_b2_rdata, way0_b3_rdata;
wire [31:0] way1_b0_rdata, way1_b1_rdata, way1_b2_rdata, way1_b3_rdata;
wire [31:0] way2_b0_rdata, way2_b1_rdata, way2_b2_rdata, way2_b3_rdata;
wire [31:0] way3_b0_rdata, way3_b1_rdata, way3_b2_rdata, way3_b3_rdata;

wire [127:0] way0_line = {way0_b3_rdata, way0_b2_rdata, way0_b1_rdata, way0_b0_rdata};
wire [127:0] way1_line = {way1_b3_rdata, way1_b2_rdata, way1_b1_rdata, way1_b0_rdata};
wire [127:0] way2_line = {way2_b3_rdata, way2_b2_rdata, way2_b1_rdata, way2_b0_rdata};
wire [127:0] way3_line = {way3_b3_rdata, way3_b2_rdata, way3_b1_rdata, way3_b0_rdata};

wire [127:0] hit_line = (hit_way == 2'd0) ? way0_line :
                        (hit_way == 2'd1) ? way1_line :
                        (hit_way == 2'd2) ? way2_line : way3_line;
wire [31:0] hit_word  = select_word(hit_line, req_offset[3:2]);
wire [31:0] hit_word2 = select_word(hit_line, req_offset[3:2] + 2'd1);
wire [127:0] hur_hit_line = (hur_hit_way == 2'd0) ? way0_line :
                            (hur_hit_way == 2'd1) ? way1_line :
                            (hur_hit_way == 2'd2) ? way2_line : way3_line;
wire [31:0] hur_hit_word = select_word(hur_hit_line, hur_offset[3:2]);

// Invalid-first, then round-robin. The 2-way configuration uses only bit 0.
wire [1:0] choose_way = !way0_valid ? 2'd0 :
                        !way1_valid ? 2'd1 :
                        (FOUR_WAY && !way2_valid) ? 2'd2 :
                        (FOUR_WAY && !way3_valid) ? 2'd3 :
                        (FOUR_WAY ? random_way : {1'b0, random_way[0]});
wire [20:0] choose_tagv = (choose_way == 2'd0) ? tagv_way0_rdata :
                          (choose_way == 2'd1) ? tagv_way1_rdata :
                          (choose_way == 2'd2) ? tagv_way2_rdata : tagv_way3_rdata;
wire choose_valid = choose_tagv[0];
wire choose_dirty = (choose_way == 2'd0) ? dirty_way0[req_index] :
                    (choose_way == 2'd1) ? dirty_way1[req_index] :
                    (choose_way == 2'd2) ? dirty_way2[req_index] : dirty_way3[req_index];
wire [127:0] choose_line = (choose_way == 2'd0) ? way0_line :
                           (choose_way == 2'd1) ? way1_line :
                           (choose_way == 2'd2) ? way2_line : way3_line;
wire [127:0] cacop_tgt_line = (cacop_target_way == 2'd0) ? way0_line :
                              (cacop_target_way == 2'd1) ? way1_line :
                              (cacop_target_way == 2'd2) ? way2_line : way3_line;

// The adjacent-set tag outputs are valid throughout the second half of the
// 8-beat transaction. Never install over an existing copy (which may be
// dirty), and never evict a dirty victim merely for speculative data.
wire adjacent_way0_hit = way0_valid && (way0_tag == prefetch_tag);
wire adjacent_way1_hit = way1_valid && (way1_tag == prefetch_tag);
wire adjacent_way2_hit = way2_valid && (way2_tag == prefetch_tag);
wire adjacent_way3_hit = way3_valid && (way3_tag == prefetch_tag);
wire adjacent_cache_hit = adjacent_way0_hit || adjacent_way1_hit ||
                          adjacent_way2_hit || adjacent_way3_hit;
assign adjacent_choose_way = !way0_valid ? 2'd0 :
                             !way1_valid ? 2'd1 :
                             (FOUR_WAY && !way2_valid) ? 2'd2 :
                             (FOUR_WAY && !way3_valid) ? 2'd3 :
                             (FOUR_WAY ? random_way : {1'b0, random_way[0]});
wire adjacent_has_invalid = !way0_valid || !way1_valid ||
                            (FOUR_WAY && (!way2_valid || !way3_valid));

// M19.10 speculative-install timing cut.
//
// Tag/data RAMs have already addressed the speculative set for multiple beats
// before a sector completes.  Sample the hit/invalid-way decision on the
// penultimate beat, then use only the two small registers on the final-beat
// BRAM write controls.  The old final-beat expression placed req_index
// increment, tag comparison, victim selection and every BRAM DI/WE pin in one
// setup path.
wire adjacent_probe_capture =
     ADJACENT_LINE_FILL && (state == S_REFILL) && ret_valid &&
     ((ret_cnt == 5'd6) || (ret_cnt == 5'd10) ||
      ((FOUR_LINE_FILL || FIVE_LINE_FILL) && (ret_cnt == 5'd14)) ||
      (FIVE_LINE_FILL && (ret_cnt == 5'd18)));
wire adjacent_install_candidate =
     !adjacent_cache_hit && (adjacent_has_invalid || READ_ONLY_CACHE);

always @(posedge clk) begin
    if (!resetn) begin
        adjacent_install_armed <= 1'b0;
        adjacent_choose_way_r  <= 2'b0;
    end
    else if (state != S_REFILL) begin
        adjacent_install_armed <= 1'b0;
    end
    else if (adjacent_probe_capture) begin
        adjacent_install_armed <= adjacent_install_candidate;
        adjacent_choose_way_r  <= adjacent_choose_way;
    end
    else if (adjacent_install) begin
        adjacent_install_armed <= 1'b0;
    end
end

assign adjacent_install = ADJACENT_LINE_FILL && (state == S_REFILL) &&
                          ret_valid &&
                          ((ret_cnt == 5'd7) || (ret_cnt == 5'd11) ||
                           (FOUR_LINE_FILL && (ret_cnt == 5'd15)) ||
                           (FIVE_LINE_FILL &&
                            ((ret_cnt == 5'd15) || ret_last))) &&
                          adjacent_install_armed;

// Write-hit and refill paths
wire hit_write = (state == S_LOOKUP) && cache_hit && req_op;
wire hit_write_way0 = hit_write && hit_way == 2'd0;
wire hit_write_way1 = hit_write && hit_way == 2'd1;
wire hit_write_way2 = hit_write && hit_way == 2'd2;
wire hit_write_way3 = hit_write && hit_way == 2'd3;
wire [1:0] hit_bank = req_offset[3:2];

// One explicit write port per dirty RAM.  All dirty-state update events are
// mutually exclusive by cache state; priority only documents the address/data
// selection for synthesis and keeps the RAM template unambiguous.
wire [9:0] dirty_write_index = tagv_init ? init_index :
                               adjacent_install ? prefetch_index : req_index;
wire dirty_write_data = hit_write ? 1'b1 :
                        refill_last ? req_op : 1'b0;
wire dirty_we0 = tagv_init || hit_write_way0 ||
                 (refill_last && repl_way == 2'd0) ||
                 (adjacent_install && adjacent_choose_way_r == 2'd0) ||
                 cacop_clear_way0;
wire dirty_we1 = tagv_init || hit_write_way1 ||
                 (refill_last && repl_way == 2'd1) ||
                 (adjacent_install && adjacent_choose_way_r == 2'd1) ||
                 cacop_clear_way1;
wire dirty_we2 = tagv_init || hit_write_way2 ||
                 (refill_last && repl_way == 2'd2) ||
                 (adjacent_install && adjacent_choose_way_r == 2'd2) ||
                 cacop_clear_way2;
wire dirty_we3 = tagv_init || hit_write_way3 ||
                 (refill_last && repl_way == 2'd3) ||
                 (adjacent_install && adjacent_choose_way_r == 2'd3) ||
                 cacop_clear_way3;

always @(posedge clk) begin
    if (dirty_we0) dirty_way0[dirty_write_index] <= dirty_write_data;
    if (dirty_we1) dirty_way1[dirty_write_index] <= dirty_write_data;
    if (dirty_we2) dirty_way2[dirty_write_index] <= dirty_write_data;
    if (dirty_we3) dirty_way3[dirty_write_index] <= dirty_write_data;
end

wire refill_beat_write = (state == S_REFILL) && ret_valid && !HIT_UNDER_REFILL &&
                         (!ADJACENT_LINE_FILL || demand_refill_phase);
wire refill_commit     = refill_last && HIT_UNDER_REFILL;
wire refill_write_way0 = (refill_beat_write || refill_commit) && repl_way == 2'd0;
wire refill_write_way1 = (refill_beat_write || refill_commit) && repl_way == 2'd1;
wire refill_write_way2 = (refill_beat_write || refill_commit) && repl_way == 2'd2;
wire refill_write_way3 = (refill_beat_write || refill_commit) && repl_way == 2'd3;
wire [1:0] refill_bank = ret_cnt[1:0];
wire [31:0] refill_word = (req_op && demand_refill_phase &&
                           ret_cnt[1:0] == req_offset[3:2]) ?
                          merge_word(ret_data, req_wdata, req_wstrb) : ret_data;
wire [127:0] refill_line_next =
                   insert_word(refill_buf, ret_cnt[1:0], refill_word);
wire [127:0] adjacent_refill_line_next =
                   insert_word(fourth_refill_phase ? fourth_refill_buf :
                               third_refill_phase  ? third_refill_buf  :
                               stride_refill_phase ? stride_refill_buf :
                                                     adjacent_refill_buf,
                               ret_cnt[1:0], ret_data);
wire hur_word_ready_now = hur_same_line &&
                          (refill_valid_mask[hur_offset[3:2]] ||
                           (ret_valid && demand_refill_phase &&
                            (ret_cnt[1:0] == hur_offset[3:2])));
wire [31:0] hur_refill_word = (ret_valid && demand_refill_phase &&
                               (ret_cnt[1:0] == hur_offset[3:2])) ?
                              refill_word :
                              select_word(refill_buf, hur_offset[3:2]);

// Per-bank write enables
wire [3:0] w0b0_we = (hit_write_way0 && hit_bank == 2'd0) ? req_wstrb :
                     (refill_commit && repl_way == 2'd0) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd0) ? 4'hf :
                     (refill_beat_write && refill_write_way0 && refill_bank == 2'd0) ? 4'hf : 4'h0;
wire [3:0] w0b1_we = (hit_write_way0 && hit_bank == 2'd1) ? req_wstrb :
                     (refill_commit && repl_way == 2'd0) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd0) ? 4'hf :
                     (refill_beat_write && refill_write_way0 && refill_bank == 2'd1) ? 4'hf : 4'h0;
wire [3:0] w0b2_we = (hit_write_way0 && hit_bank == 2'd2) ? req_wstrb :
                     (refill_commit && repl_way == 2'd0) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd0) ? 4'hf :
                     (refill_beat_write && refill_write_way0 && refill_bank == 2'd2) ? 4'hf : 4'h0;
wire [3:0] w0b3_we = (hit_write_way0 && hit_bank == 2'd3) ? req_wstrb :
                     (refill_commit && repl_way == 2'd0) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd0) ? 4'hf :
                     (refill_beat_write && refill_write_way0 && refill_bank == 2'd3) ? 4'hf : 4'h0;
wire [3:0] w1b0_we = (hit_write_way1 && hit_bank == 2'd0) ? req_wstrb :
                     (refill_commit && repl_way == 2'd1) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd1) ? 4'hf :
                     (refill_beat_write && refill_write_way1 && refill_bank == 2'd0) ? 4'hf : 4'h0;
wire [3:0] w1b1_we = (hit_write_way1 && hit_bank == 2'd1) ? req_wstrb :
                     (refill_commit && repl_way == 2'd1) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd1) ? 4'hf :
                     (refill_beat_write && refill_write_way1 && refill_bank == 2'd1) ? 4'hf : 4'h0;
wire [3:0] w1b2_we = (hit_write_way1 && hit_bank == 2'd2) ? req_wstrb :
                     (refill_commit && repl_way == 2'd1) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd1) ? 4'hf :
                     (refill_beat_write && refill_write_way1 && refill_bank == 2'd2) ? 4'hf : 4'h0;
wire [3:0] w1b3_we = (hit_write_way1 && hit_bank == 2'd3) ? req_wstrb :
                     (refill_commit && repl_way == 2'd1) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd1) ? 4'hf :
                     (refill_beat_write && refill_write_way1 && refill_bank == 2'd3) ? 4'hf : 4'h0;
wire [3:0] w2b0_we = (hit_write_way2 && hit_bank == 2'd0) ? req_wstrb :
                     (refill_commit && repl_way == 2'd2) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd2) ? 4'hf :
                     (refill_beat_write && refill_write_way2 && refill_bank == 2'd0) ? 4'hf : 4'h0;
wire [3:0] w2b1_we = (hit_write_way2 && hit_bank == 2'd1) ? req_wstrb :
                     (refill_commit && repl_way == 2'd2) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd2) ? 4'hf :
                     (refill_beat_write && refill_write_way2 && refill_bank == 2'd1) ? 4'hf : 4'h0;
wire [3:0] w2b2_we = (hit_write_way2 && hit_bank == 2'd2) ? req_wstrb :
                     (refill_commit && repl_way == 2'd2) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd2) ? 4'hf :
                     (refill_beat_write && refill_write_way2 && refill_bank == 2'd2) ? 4'hf : 4'h0;
wire [3:0] w2b3_we = (hit_write_way2 && hit_bank == 2'd3) ? req_wstrb :
                     (refill_commit && repl_way == 2'd2) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd2) ? 4'hf :
                     (refill_beat_write && refill_write_way2 && refill_bank == 2'd3) ? 4'hf : 4'h0;
wire [3:0] w3b0_we = (hit_write_way3 && hit_bank == 2'd0) ? req_wstrb :
                     (refill_commit && repl_way == 2'd3) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd3) ? 4'hf :
                     (refill_beat_write && refill_write_way3 && refill_bank == 2'd0) ? 4'hf : 4'h0;
wire [3:0] w3b1_we = (hit_write_way3 && hit_bank == 2'd1) ? req_wstrb :
                     (refill_commit && repl_way == 2'd3) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd3) ? 4'hf :
                     (refill_beat_write && refill_write_way3 && refill_bank == 2'd1) ? 4'hf : 4'h0;
wire [3:0] w3b2_we = (hit_write_way3 && hit_bank == 2'd2) ? req_wstrb :
                     (refill_commit && repl_way == 2'd3) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd3) ? 4'hf :
                     (refill_beat_write && refill_write_way3 && refill_bank == 2'd2) ? 4'hf : 4'h0;
wire [3:0] w3b3_we = (hit_write_way3 && hit_bank == 2'd3) ? req_wstrb :
                     (refill_commit && repl_way == 2'd3) ? 4'hf :
                     (adjacent_install && adjacent_choose_way_r == 2'd3) ? 4'hf :
                     (refill_beat_write && refill_write_way3 && refill_bank == 2'd3) ? 4'hf : 4'h0;

wire [127:0] install_line = adjacent_install ? adjacent_refill_line_next : refill_line_next;
wire install_way0 = (refill_commit && repl_way == 2'd0) ||
                    (adjacent_install && adjacent_choose_way_r == 2'd0);
wire install_way1 = (refill_commit && repl_way == 2'd1) ||
                    (adjacent_install && adjacent_choose_way_r == 2'd1);
wire install_way2 = (refill_commit && repl_way == 2'd2) ||
                    (adjacent_install && adjacent_choose_way_r == 2'd2);
wire install_way3 = (refill_commit && repl_way == 2'd3) ||
                    (adjacent_install && adjacent_choose_way_r == 2'd3);

wire [31:0] w0b0_din = install_way0 ?
                            select_word(install_line, 2'd0) :
                            (refill_beat_write && refill_write_way0 && refill_bank == 2'd0) ?
                            refill_word : req_wdata;
wire [31:0] w0b1_din = install_way0 ?
                            select_word(install_line, 2'd1) :
                            (refill_beat_write && refill_write_way0 && refill_bank == 2'd1) ?
                            refill_word : req_wdata;
wire [31:0] w0b2_din = install_way0 ?
                            select_word(install_line, 2'd2) :
                            (refill_beat_write && refill_write_way0 && refill_bank == 2'd2) ?
                            refill_word : req_wdata;
wire [31:0] w0b3_din = install_way0 ?
                            select_word(install_line, 2'd3) :
                            (refill_beat_write && refill_write_way0 && refill_bank == 2'd3) ?
                            refill_word : req_wdata;
wire [31:0] w1b0_din = install_way1 ?
                            select_word(install_line, 2'd0) :
                            (refill_beat_write && refill_write_way1 && refill_bank == 2'd0) ?
                            refill_word : req_wdata;
wire [31:0] w1b1_din = install_way1 ?
                            select_word(install_line, 2'd1) :
                            (refill_beat_write && refill_write_way1 && refill_bank == 2'd1) ?
                            refill_word : req_wdata;
wire [31:0] w1b2_din = install_way1 ?
                            select_word(install_line, 2'd2) :
                            (refill_beat_write && refill_write_way1 && refill_bank == 2'd2) ?
                            refill_word : req_wdata;
wire [31:0] w1b3_din = install_way1 ?
                            select_word(install_line, 2'd3) :
                            (refill_beat_write && refill_write_way1 && refill_bank == 2'd3) ?
                            refill_word : req_wdata;
wire [31:0] w2b0_din = install_way2 ?
                            select_word(install_line, 2'd0) :
                            (refill_beat_write && refill_write_way2 && refill_bank == 2'd0) ?
                            refill_word : req_wdata;
wire [31:0] w2b1_din = install_way2 ?
                            select_word(install_line, 2'd1) :
                            (refill_beat_write && refill_write_way2 && refill_bank == 2'd1) ?
                            refill_word : req_wdata;
wire [31:0] w2b2_din = install_way2 ?
                            select_word(install_line, 2'd2) :
                            (refill_beat_write && refill_write_way2 && refill_bank == 2'd2) ?
                            refill_word : req_wdata;
wire [31:0] w2b3_din = install_way2 ?
                            select_word(install_line, 2'd3) :
                            (refill_beat_write && refill_write_way2 && refill_bank == 2'd3) ?
                            refill_word : req_wdata;
wire [31:0] w3b0_din = install_way3 ?
                            select_word(install_line, 2'd0) :
                            (refill_beat_write && refill_write_way3 && refill_bank == 2'd0) ?
                            refill_word : req_wdata;
wire [31:0] w3b1_din = install_way3 ?
                            select_word(install_line, 2'd1) :
                            (refill_beat_write && refill_write_way3 && refill_bank == 2'd1) ?
                            refill_word : req_wdata;
wire [31:0] w3b2_din = install_way3 ?
                            select_word(install_line, 2'd2) :
                            (refill_beat_write && refill_write_way3 && refill_bank == 2'd2) ?
                            refill_word : req_wdata;
wire [31:0] w3b3_din = install_way3 ?
                            select_word(install_line, 2'd3) :
                            (refill_beat_write && refill_write_way3 && refill_bank == 2'd3) ?
                            refill_word : req_wdata;

generate
    if (SETS_1024) begin : gen_1024_data
        m184_data_bank_sram_1024 u_data_way0_bank0 (.clka(clk), .ena(1'b1), .wea(w0b0_we), .addra(ram_addr), .dina(w0b0_din), .douta(way0_b0_rdata));
        m184_data_bank_sram_1024 u_data_way0_bank1 (.clka(clk), .ena(1'b1), .wea(w0b1_we), .addra(ram_addr), .dina(w0b1_din), .douta(way0_b1_rdata));
        m184_data_bank_sram_1024 u_data_way0_bank2 (.clka(clk), .ena(1'b1), .wea(w0b2_we), .addra(ram_addr), .dina(w0b2_din), .douta(way0_b2_rdata));
        m184_data_bank_sram_1024 u_data_way0_bank3 (.clka(clk), .ena(1'b1), .wea(w0b3_we), .addra(ram_addr), .dina(w0b3_din), .douta(way0_b3_rdata));
        m184_data_bank_sram_1024 u_data_way1_bank0 (.clka(clk), .ena(1'b1), .wea(w1b0_we), .addra(ram_addr), .dina(w1b0_din), .douta(way1_b0_rdata));
        m184_data_bank_sram_1024 u_data_way1_bank1 (.clka(clk), .ena(1'b1), .wea(w1b1_we), .addra(ram_addr), .dina(w1b1_din), .douta(way1_b1_rdata));
        m184_data_bank_sram_1024 u_data_way1_bank2 (.clka(clk), .ena(1'b1), .wea(w1b2_we), .addra(ram_addr), .dina(w1b2_din), .douta(way1_b2_rdata));
        m184_data_bank_sram_1024 u_data_way1_bank3 (.clka(clk), .ena(1'b1), .wea(w1b3_we), .addra(ram_addr), .dina(w1b3_din), .douta(way1_b3_rdata));
        if (FOUR_WAY) begin : gen_four_way_data_1024
            m184_data_bank_sram_1024 u_data_way2_bank0 (.clka(clk), .ena(1'b1), .wea(w2b0_we), .addra(ram_addr), .dina(w2b0_din), .douta(way2_b0_rdata));
            m184_data_bank_sram_1024 u_data_way2_bank1 (.clka(clk), .ena(1'b1), .wea(w2b1_we), .addra(ram_addr), .dina(w2b1_din), .douta(way2_b1_rdata));
            m184_data_bank_sram_1024 u_data_way2_bank2 (.clka(clk), .ena(1'b1), .wea(w2b2_we), .addra(ram_addr), .dina(w2b2_din), .douta(way2_b2_rdata));
            m184_data_bank_sram_1024 u_data_way2_bank3 (.clka(clk), .ena(1'b1), .wea(w2b3_we), .addra(ram_addr), .dina(w2b3_din), .douta(way2_b3_rdata));
            m184_data_bank_sram_1024 u_data_way3_bank0 (.clka(clk), .ena(1'b1), .wea(w3b0_we), .addra(ram_addr), .dina(w3b0_din), .douta(way3_b0_rdata));
            m184_data_bank_sram_1024 u_data_way3_bank1 (.clka(clk), .ena(1'b1), .wea(w3b1_we), .addra(ram_addr), .dina(w3b1_din), .douta(way3_b1_rdata));
            m184_data_bank_sram_1024 u_data_way3_bank2 (.clka(clk), .ena(1'b1), .wea(w3b2_we), .addra(ram_addr), .dina(w3b2_din), .douta(way3_b2_rdata));
            m184_data_bank_sram_1024 u_data_way3_bank3 (.clka(clk), .ena(1'b1), .wea(w3b3_we), .addra(ram_addr), .dina(w3b3_din), .douta(way3_b3_rdata));
        end
        else begin : gen_two_way_data_1024
            assign way2_b0_rdata = 32'b0;
            assign way2_b1_rdata = 32'b0;
            assign way2_b2_rdata = 32'b0;
            assign way2_b3_rdata = 32'b0;
            assign way3_b0_rdata = 32'b0;
            assign way3_b1_rdata = 32'b0;
            assign way3_b2_rdata = 32'b0;
            assign way3_b3_rdata = 32'b0;
        end
    end
    else if (SETS_512) begin : gen_512_data
        m175d_data_bank_sram_512 u_data_way0_bank0 (.clka(clk), .ena(1'b1), .wea(w0b0_we), .addra(ram_addr[8:0]), .dina(w0b0_din), .douta(way0_b0_rdata));
        m175d_data_bank_sram_512 u_data_way0_bank1 (.clka(clk), .ena(1'b1), .wea(w0b1_we), .addra(ram_addr[8:0]), .dina(w0b1_din), .douta(way0_b1_rdata));
        m175d_data_bank_sram_512 u_data_way0_bank2 (.clka(clk), .ena(1'b1), .wea(w0b2_we), .addra(ram_addr[8:0]), .dina(w0b2_din), .douta(way0_b2_rdata));
        m175d_data_bank_sram_512 u_data_way0_bank3 (.clka(clk), .ena(1'b1), .wea(w0b3_we), .addra(ram_addr[8:0]), .dina(w0b3_din), .douta(way0_b3_rdata));
        m175d_data_bank_sram_512 u_data_way1_bank0 (.clka(clk), .ena(1'b1), .wea(w1b0_we), .addra(ram_addr[8:0]), .dina(w1b0_din), .douta(way1_b0_rdata));
        m175d_data_bank_sram_512 u_data_way1_bank1 (.clka(clk), .ena(1'b1), .wea(w1b1_we), .addra(ram_addr[8:0]), .dina(w1b1_din), .douta(way1_b1_rdata));
        m175d_data_bank_sram_512 u_data_way1_bank2 (.clka(clk), .ena(1'b1), .wea(w1b2_we), .addra(ram_addr[8:0]), .dina(w1b2_din), .douta(way1_b2_rdata));
        m175d_data_bank_sram_512 u_data_way1_bank3 (.clka(clk), .ena(1'b1), .wea(w1b3_we), .addra(ram_addr[8:0]), .dina(w1b3_din), .douta(way1_b3_rdata));
        if (FOUR_WAY) begin : gen_four_way_data_512
            m175d_data_bank_sram_512 u_data_way2_bank0 (.clka(clk), .ena(1'b1), .wea(w2b0_we), .addra(ram_addr[8:0]), .dina(w2b0_din), .douta(way2_b0_rdata));
            m175d_data_bank_sram_512 u_data_way2_bank1 (.clka(clk), .ena(1'b1), .wea(w2b1_we), .addra(ram_addr[8:0]), .dina(w2b1_din), .douta(way2_b1_rdata));
            m175d_data_bank_sram_512 u_data_way2_bank2 (.clka(clk), .ena(1'b1), .wea(w2b2_we), .addra(ram_addr[8:0]), .dina(w2b2_din), .douta(way2_b2_rdata));
            m175d_data_bank_sram_512 u_data_way2_bank3 (.clka(clk), .ena(1'b1), .wea(w2b3_we), .addra(ram_addr[8:0]), .dina(w2b3_din), .douta(way2_b3_rdata));
            m175d_data_bank_sram_512 u_data_way3_bank0 (.clka(clk), .ena(1'b1), .wea(w3b0_we), .addra(ram_addr[8:0]), .dina(w3b0_din), .douta(way3_b0_rdata));
            m175d_data_bank_sram_512 u_data_way3_bank1 (.clka(clk), .ena(1'b1), .wea(w3b1_we), .addra(ram_addr[8:0]), .dina(w3b1_din), .douta(way3_b1_rdata));
            m175d_data_bank_sram_512 u_data_way3_bank2 (.clka(clk), .ena(1'b1), .wea(w3b2_we), .addra(ram_addr[8:0]), .dina(w3b2_din), .douta(way3_b2_rdata));
            m175d_data_bank_sram_512 u_data_way3_bank3 (.clka(clk), .ena(1'b1), .wea(w3b3_we), .addra(ram_addr[8:0]), .dina(w3b3_din), .douta(way3_b3_rdata));
        end
        else begin : gen_two_way_data_512
            assign way2_b0_rdata = 32'b0;
            assign way2_b1_rdata = 32'b0;
            assign way2_b2_rdata = 32'b0;
            assign way2_b3_rdata = 32'b0;
            assign way3_b0_rdata = 32'b0;
            assign way3_b1_rdata = 32'b0;
            assign way3_b2_rdata = 32'b0;
            assign way3_b3_rdata = 32'b0;
        end
    end
    else begin : gen_256_data
        data_bank_sram u_data_way0_bank0 (.clka(clk), .ena(1'b1), .wea(w0b0_we), .addra(ram_addr[7:0]), .dina(w0b0_din), .douta(way0_b0_rdata));
        data_bank_sram u_data_way0_bank1 (.clka(clk), .ena(1'b1), .wea(w0b1_we), .addra(ram_addr[7:0]), .dina(w0b1_din), .douta(way0_b1_rdata));
        data_bank_sram u_data_way0_bank2 (.clka(clk), .ena(1'b1), .wea(w0b2_we), .addra(ram_addr[7:0]), .dina(w0b2_din), .douta(way0_b2_rdata));
        data_bank_sram u_data_way0_bank3 (.clka(clk), .ena(1'b1), .wea(w0b3_we), .addra(ram_addr[7:0]), .dina(w0b3_din), .douta(way0_b3_rdata));
        data_bank_sram u_data_way1_bank0 (.clka(clk), .ena(1'b1), .wea(w1b0_we), .addra(ram_addr[7:0]), .dina(w1b0_din), .douta(way1_b0_rdata));
        data_bank_sram u_data_way1_bank1 (.clka(clk), .ena(1'b1), .wea(w1b1_we), .addra(ram_addr[7:0]), .dina(w1b1_din), .douta(way1_b1_rdata));
        data_bank_sram u_data_way1_bank2 (.clka(clk), .ena(1'b1), .wea(w1b2_we), .addra(ram_addr[7:0]), .dina(w1b2_din), .douta(way1_b2_rdata));
        data_bank_sram u_data_way1_bank3 (.clka(clk), .ena(1'b1), .wea(w1b3_we), .addra(ram_addr[7:0]), .dina(w1b3_din), .douta(way1_b3_rdata));
        if (FOUR_WAY) begin : gen_four_way_data
            data_bank_sram u_data_way2_bank0 (.clka(clk), .ena(1'b1), .wea(w2b0_we), .addra(ram_addr[7:0]), .dina(w2b0_din), .douta(way2_b0_rdata));
            data_bank_sram u_data_way2_bank1 (.clka(clk), .ena(1'b1), .wea(w2b1_we), .addra(ram_addr[7:0]), .dina(w2b1_din), .douta(way2_b1_rdata));
            data_bank_sram u_data_way2_bank2 (.clka(clk), .ena(1'b1), .wea(w2b2_we), .addra(ram_addr[7:0]), .dina(w2b2_din), .douta(way2_b2_rdata));
            data_bank_sram u_data_way2_bank3 (.clka(clk), .ena(1'b1), .wea(w2b3_we), .addra(ram_addr[7:0]), .dina(w2b3_din), .douta(way2_b3_rdata));
            data_bank_sram u_data_way3_bank0 (.clka(clk), .ena(1'b1), .wea(w3b0_we), .addra(ram_addr[7:0]), .dina(w3b0_din), .douta(way3_b0_rdata));
            data_bank_sram u_data_way3_bank1 (.clka(clk), .ena(1'b1), .wea(w3b1_we), .addra(ram_addr[7:0]), .dina(w3b1_din), .douta(way3_b1_rdata));
            data_bank_sram u_data_way3_bank2 (.clka(clk), .ena(1'b1), .wea(w3b2_we), .addra(ram_addr[7:0]), .dina(w3b2_din), .douta(way3_b2_rdata));
            data_bank_sram u_data_way3_bank3 (.clka(clk), .ena(1'b1), .wea(w3b3_we), .addra(ram_addr[7:0]), .dina(w3b3_din), .douta(way3_b3_rdata));
        end
        else begin : gen_two_way_data
            assign way2_b0_rdata = 32'b0;
            assign way2_b1_rdata = 32'b0;
            assign way2_b2_rdata = 32'b0;
            assign way2_b3_rdata = 32'b0;
            assign way3_b0_rdata = 32'b0;
            assign way3_b1_rdata = 32'b0;
            assign way3_b2_rdata = 32'b0;
            assign way3_b3_rdata = 32'b0;
        end
    end
endgenerate

// ------------------------------------------------------------
// External interface outputs
// ------------------------------------------------------------
assign addr_ok = !cacop_valid &&
                 (((state == S_IDLE) && !hur_replay_setup) ||
                  (PIPELINED_HIT && state == S_LOOKUP && cache_hit && !req_op) ||
                  hur_addr_window);
assign cacop_addr_ok = cacop_accept;
assign rd_req   = (state == S_RDREQ);
assign rd_type  = FIVE_LINE_FILL ? TYPE_FIVE_LINE :
                  FOUR_LINE_FILL ? TYPE_FOUR_LINE :
                  ADJACENT_LINE_FILL ? TYPE_TRIPLE_LINE : TYPE_LINE;
assign rd_addr  = compose_line_addr(req_tag, req_index);
assign wr_req   = (state == S_WB) || (state == S_CACOP_WB);
assign wr_type  = TYPE_LINE;
assign wr_addr  = compose_line_addr(repl_old_tag, req_index);
assign wr_wstrb = 4'b1111;
assign wr_data  = repl_old_data;

// ------------------------------------------------------------
// Main FSM
// ------------------------------------------------------------
always @(posedge clk) begin
    if (!resetn) begin
        state      <= S_INIT;
        init_index <= 10'b0;
        data_ok       <= 1'b0;
        cacop_data_ok <= 1'b0;
        rdata         <= 32'b0;
        rdata2        <= 32'b0;

        req_op          <= 1'b0;
        req_index       <= 10'b0;
        req_tag         <= 20'b0;
        req_offset      <= 4'b0;
        req_wstrb       <= 4'b0;
        req_wdata       <= 32'b0;
        req_dual_read   <= 1'b0;
        req_cacop_code  <= 5'b0;
        req_cacop_way   <= 2'b0;

        repl_way       <= 2'b0;
        repl_old_tag   <= 20'b0;
        repl_old_data  <= 128'b0;
        refill_buf     <= 128'b0;
        adjacent_refill_buf <= 128'b0;
        stride_refill_buf <= 128'b0;
        third_refill_buf <= 128'b0;
        fourth_refill_buf <= 128'b0;
        ret_cnt        <= 5'b0;
        random_way       <= 2'b0;
        miss_resp_sent   <= 1'b0;
        hur_valid         <= 1'b0;
        hur_probe_pending <= 1'b0;
        hur_second_miss   <= 1'b0;
        hur_replay_setup  <= 1'b0;
        hur_replay_active <= 1'b0;
        hur_index         <= 10'b0;
        hur_tag           <= 20'b0;
        hur_offset        <= 4'b0;
        refill_valid_mask <= 4'b0;
`ifndef SYNTHESIS
        hur_accept_cnt          <= 32'b0;
        hur_same_line_hit_cnt   <= 32'b0;
        hur_other_line_hit_cnt  <= 32'b0;
        hur_second_miss_cnt     <= 32'b0;
        hur_word_wait_cycle_cnt <= 32'b0;
        hur_store_block_cycle_cnt <= 32'b0;
        hur_replay_setup_cnt      <= 32'b0;
        hur_replay_hit_cnt        <= 32'b0;
        hur_replay_miss_cnt       <= 32'b0;
        hur_debug_cycle_cnt       <= 32'b0;
        hur_debug_event_seq       <= 32'b0;
        sim_lookup_cnt            <= 32'b0;
        sim_hit_cnt               <= 32'b0;
        sim_miss_cnt              <= 32'b0;
        sim_refill_line_cnt       <= 32'b0;
        sim_rdreq_cycle_cnt       <= 32'b0;
        sim_refill_cycle_cnt      <= 32'b0;
        sim_way0_hit_cnt          <= 32'b0;
        sim_way1_hit_cnt          <= 32'b0;
        sim_way2_hit_cnt          <= 32'b0;
        sim_way3_hit_cnt          <= 32'b0;
        sim_way0_victim_cnt       <= 32'b0;
        sim_way1_victim_cnt       <= 32'b0;
        sim_way2_victim_cnt       <= 32'b0;
        sim_way3_victim_cnt       <= 32'b0;
`endif
    end
    else begin
        data_ok       <= 1'b0;
        cacop_data_ok <= 1'b0;
`ifndef SYNTHESIS
        hur_debug_cycle_cnt <= hur_debug_cycle_cnt + 32'd1;
        if (HIT_UNDER_REFILL && state == S_REFILL && miss_resp_sent && valid && op)
            hur_store_block_cycle_cnt <= hur_store_block_cycle_cnt + 32'd1;
        if (HIT_UNDER_REFILL && state == S_REFILL && hur_valid && hur_same_line &&
            !hur_word_ready_now)
            hur_word_wait_cycle_cnt <= hur_word_wait_cycle_cnt + 32'd1;

        if (state == S_RDREQ)
            sim_rdreq_cycle_cnt <= sim_rdreq_cycle_cnt + 32'd1;
        if (state == S_REFILL)
            sim_refill_cycle_cnt <= sim_refill_cycle_cnt + 32'd1;
        if (refill_last)
            sim_refill_line_cnt <= sim_refill_line_cnt + 32'd1;
        if (state == S_LOOKUP) begin
            sim_lookup_cnt <= sim_lookup_cnt + 32'd1;
            if (cache_hit) begin
                sim_hit_cnt <= sim_hit_cnt + 32'd1;
                case (hit_way)
                    2'd0: sim_way0_hit_cnt <= sim_way0_hit_cnt + 32'd1;
                    2'd1: sim_way1_hit_cnt <= sim_way1_hit_cnt + 32'd1;
                    2'd2: sim_way2_hit_cnt <= sim_way2_hit_cnt + 32'd1;
                    default: sim_way3_hit_cnt <= sim_way3_hit_cnt + 32'd1;
                endcase
            end else begin
                sim_miss_cnt <= sim_miss_cnt + 32'd1;
                case (choose_way)
                    2'd0: sim_way0_victim_cnt <= sim_way0_victim_cnt + 32'd1;
                    2'd1: sim_way1_victim_cnt <= sim_way1_victim_cnt + 32'd1;
                    2'd2: sim_way2_victim_cnt <= sim_way2_victim_cnt + 32'd1;
                    default: sim_way3_victim_cnt <= sim_way3_victim_cnt + 32'd1;
                endcase
            end
        end
`endif

        case (state)
            S_INIT: begin
                if (init_index == LAST_INDEX)
                    state <= S_IDLE;
                init_index <= init_index + 10'b1;
            end

            S_IDLE: begin
                // A replayed second miss needs one full RAM-address setup
                // cycle before S_LOOKUP.  The previous M17.3N implementation
                // jumped directly from refill completion to S_LOOKUP, so the
                // synchronous tag/data RAM outputs still described the old
                // refill set.  This explicit setup cycle makes the replay use
                // the correct set on the following lookup edge.
                if (hur_replay_setup) begin
                    hur_replay_setup  <= 1'b0;
                    hur_replay_active <= 1'b1;
                    state             <= S_LOOKUP;
`ifndef SYNTHESIS
                    if (hur_debug_event_seq < 32'd16) begin
                        $display("HUR_EVENT|kind=replay_probe|seq=%0d|cycle=%0d|addr=0x%08x|index=0x%03x|tag=0x%05x|offset=%0d",
                                 hur_debug_event_seq, hur_debug_cycle_cnt,
                                 compose_byte_addr(req_tag, req_index, req_offset), req_index,
                                 req_tag, req_offset[3:2]);
                        hur_debug_event_seq <= hur_debug_event_seq + 32'd1;
                    end
`endif
                end
                else if (cacop_valid) begin
                    req_op          <= 1'b0;
                    req_index       <= cacop_index;
                    req_tag         <= cacop_tag;
                    req_offset      <= 4'b0;
                    req_wstrb       <= 4'b0;
                    req_wdata       <= 32'b0;
                    req_dual_read   <= 1'b0;
                    req_cacop_code  <= cacop_code;
                    req_cacop_way   <= cacop_way;
                    hur_valid         <= 1'b0;
                    hur_probe_pending <= 1'b0;
                    hur_second_miss   <= 1'b0;
                    hur_replay_active <= 1'b0;
                    state           <= S_CACOP;
                end
                else if (valid) begin
                    req_op          <= op;
                    req_index       <= index;
                    req_tag         <= tag;
                    req_offset      <= offset;
                    req_wstrb       <= wstrb;
                    req_wdata       <= wdata;
                    req_dual_read   <= dual_read;
                    hur_replay_active <= 1'b0;
                    state           <= S_LOOKUP;
                end
            end

            S_LOOKUP: begin
`ifndef SYNTHESIS
                if (hur_replay_active) begin
                    if (cache_hit)
                        hur_replay_hit_cnt <= hur_replay_hit_cnt + 32'd1;
                    else
                        hur_replay_miss_cnt <= hur_replay_miss_cnt + 32'd1;
                    if (hur_debug_event_seq < 32'd16) begin
                        $display("HUR_EVENT|kind=replay_lookup|seq=%0d|cycle=%0d|addr=0x%08x|hit=%0d|hit_way=%0d|word=0x%08x|way_tags=%05x,%05x,%05x,%05x",
                                 hur_debug_event_seq, hur_debug_cycle_cnt,
                                 compose_byte_addr(req_tag, req_index, req_offset), cache_hit,
                                 hit_way, hit_word, way0_tag, way1_tag,
                                 way2_tag, way3_tag);
                        hur_debug_event_seq <= hur_debug_event_seq + 32'd1;
                    end
                end
`endif
                hur_replay_active <= 1'b0;
                if (cache_hit) begin
                    if (req_op) begin
                        data_ok <= 1'b1;
                    end
                    else begin
                        rdata   <= hit_word;
                        rdata2  <= hit_word2;
                        data_ok <= 1'b1;
                    end

                    if (lookup_hit_accept) begin
                        req_op          <= op;
                        req_index       <= index;
                        req_tag         <= tag;
                        req_offset      <= offset;
                        req_wstrb       <= wstrb;
                        req_wdata       <= wdata;
                        req_dual_read   <= dual_read;
                        state           <= S_LOOKUP;
                    end
                    else begin
                        state <= S_IDLE;
                    end
                end
                else begin
                    repl_way       <= choose_way;
                    repl_old_tag   <= choose_tagv[20:1];
                    repl_old_data  <= choose_line;
                    refill_buf       <= 128'b0;
                    adjacent_refill_buf <= 128'b0;
                    stride_refill_buf <= 128'b0;
                    third_refill_buf <= 128'b0;
                    fourth_refill_buf <= 128'b0;
                    refill_valid_mask <= 4'b0;
                    ret_cnt          <= 5'b0;
                    miss_resp_sent   <= 1'b0;
                    hur_valid         <= 1'b0;
                    hur_probe_pending <= 1'b0;
                    hur_second_miss   <= 1'b0;
                    hur_replay_setup  <= 1'b0;
                    if (FOUR_WAY)
                        random_way <= random_way + 2'd1;
                    else
                        random_way <= {1'b0, ~random_way[0]};
                    state <= (choose_valid && choose_dirty) ? S_WB : S_RDREQ;
                end
            end

            S_WB: begin
                if (wr_rdy)
                    state <= S_RDREQ;
            end

            S_RDREQ: begin
                if (rd_rdy) begin
                    ret_cnt        <= 5'b0;
                    refill_buf     <= 128'b0;
                    adjacent_refill_buf <= 128'b0;
                    stride_refill_buf <= 128'b0;
                    third_refill_buf <= 128'b0;
                    fourth_refill_buf <= 128'b0;
                    miss_resp_sent <= 1'b0;
                    state           <= S_REFILL;
                end
            end

            S_REFILL: begin
                // Capture one younger cached load after the original miss has
                // produced its critical-word response.  Its RAM address is
                // presented in this same cycle; tag/data outputs are checked on
                // the following cycle.
                if (hur_accept) begin
                    hur_valid         <= 1'b1;
                    hur_index         <= index;
                    hur_tag           <= tag;
                    hur_offset        <= offset;
                    hur_probe_pending <= !((index == req_index) && (tag == req_tag));
                    hur_second_miss   <= 1'b0;
`ifndef SYNTHESIS
                    hur_accept_cnt <= hur_accept_cnt + 32'd1;
                    if (hur_debug_event_seq < 32'd16) begin
                        $display("HUR_EVENT|kind=accept|seq=%0d|cycle=%0d|active_line=0x%08x|young_addr=0x%08x|same_line=%0d|ret_mask=0x%x",
                                 hur_debug_event_seq, hur_debug_cycle_cnt,
                                 compose_line_addr(req_tag, req_index),
                                 compose_byte_addr(tag, index, offset),
                                 ((index == req_index) && (tag == req_tag)),
                                 refill_valid_mask);
                        hur_debug_event_seq <= hur_debug_event_seq + 32'd1;
                    end
`endif
                end

                // Serve a queued same-line word as soon as its return beat is
                // present, or serve an other-line resident hit from the cache
                // RAMs.  Stores are intentionally not accepted in this phase.
                if (hur_valid && hur_same_line && hur_word_ready_now && !ret_last) begin
                    rdata             <= hur_refill_word;
                    data_ok           <= 1'b1;
                    hur_valid         <= 1'b0;
                    hur_probe_pending <= 1'b0;
                    hur_second_miss   <= 1'b0;
`ifndef SYNTHESIS
                    hur_same_line_hit_cnt <= hur_same_line_hit_cnt + 32'd1;
                    if (hur_debug_event_seq < 32'd16) begin
                        $display("HUR_EVENT|kind=same_line_hit|seq=%0d|cycle=%0d|addr=0x%08x|word=0x%08x|ret_mask=0x%x",
                                 hur_debug_event_seq, hur_debug_cycle_cnt,
                                 compose_byte_addr(hur_tag, hur_index, hur_offset),
                                 hur_refill_word, refill_valid_mask);
                        hur_debug_event_seq <= hur_debug_event_seq + 32'd1;
                    end
`endif
                end
                else if (hur_valid && hur_probe_pending) begin
                    if (hur_cache_hit) begin
                        rdata             <= hur_hit_word;
                        data_ok           <= 1'b1;
                        hur_valid         <= 1'b0;
                        hur_probe_pending <= 1'b0;
                        hur_second_miss   <= 1'b0;
`ifndef SYNTHESIS
                        hur_other_line_hit_cnt <= hur_other_line_hit_cnt + 32'd1;
                        if (hur_debug_event_seq < 32'd16) begin
                            $display("HUR_EVENT|kind=other_line_hit|seq=%0d|cycle=%0d|addr=0x%08x|hit_way=%0d|word=0x%08x|way_tags=%05x,%05x,%05x,%05x",
                                     hur_debug_event_seq, hur_debug_cycle_cnt,
                                     compose_byte_addr(hur_tag, hur_index, hur_offset),
                                     hur_hit_way, hur_hit_word, way0_tag,
                                     way1_tag, way2_tag, way3_tag);
                            hur_debug_event_seq <= hur_debug_event_seq + 32'd1;
                        end
`endif
                    end
                    else begin
                        // Keep the request, but do not issue another AXI read
                        // until the active refill is completely installed.
                        hur_probe_pending <= 1'b0;
                        hur_second_miss   <= 1'b1;
`ifndef SYNTHESIS
                        hur_second_miss_cnt <= hur_second_miss_cnt + 32'd1;
                        if (hur_debug_event_seq < 32'd16) begin
                            $display("HUR_EVENT|kind=second_miss|seq=%0d|cycle=%0d|addr=0x%08x|way_tags=%05x,%05x,%05x,%05x",
                                     hur_debug_event_seq, hur_debug_cycle_cnt,
                                     compose_byte_addr(hur_tag, hur_index, hur_offset),
                                     way0_tag, way1_tag, way2_tag, way3_tag);
                            hur_debug_event_seq <= hur_debug_event_seq + 32'd1;
                        end
`endif
                    end
                end

                if (ret_valid) begin
                    if (demand_refill_phase) begin
                        refill_buf <= refill_line_next;
                        refill_valid_mask[ret_cnt[1:0]] <= 1'b1;
                    end
                    else if (adjacent_refill_phase) begin
                        adjacent_refill_buf <= adjacent_refill_line_next;
                    end
                    else if (stride_refill_phase) begin
                        stride_refill_buf <= adjacent_refill_line_next;
                    end
                    else if (third_refill_phase) begin
                        third_refill_buf <= adjacent_refill_line_next;
                    end
                    else begin
                        fourth_refill_buf <= adjacent_refill_line_next;
                    end
                    ret_cnt <= ret_cnt + 5'b00001;

                    if (!req_op && !req_dual_read && !miss_resp_sent &&
                        demand_refill_phase && ret_cnt[1:0] == req_offset[3:2]) begin
                        rdata          <= refill_word;
                        data_ok        <= 1'b1;
                        miss_resp_sent <= 1'b1;
                    end

                    // Complete the architectural miss as soon as its first
                    // 16-byte sector is present; the optional neighbour keeps
                    // using the AXI channel in the background.
                    if (refill_last) begin
                        if (!req_op && !miss_resp_sent) begin
                            rdata   <= select_word(refill_line_next, req_offset[3:2]);
                            rdata2  <= select_word(refill_line_next, req_offset[3:2] + 2'd1);
                            data_ok <= 1'b1;
                        end
                        else if (req_op) begin
                            data_ok <= 1'b1;
                        end
                    end

                    // Every speculative sector is clean.  DCache installs only
                    // into an invalid way; ICache may replace a valid clean way.
                    if (ret_last) begin
                        // A queued second miss is replayed only after the first
                        // line is installed.  A same-line request whose word is
                        // the final beat is completed here without replay.
                        if (HIT_UNDER_REFILL && hur_valid && hur_same_line &&
                            (refill_valid_mask[hur_offset[3:2]] ||
                             (demand_refill_phase && ret_cnt[1:0] == hur_offset[3:2]))) begin
                            rdata             <= (demand_refill_phase && ret_cnt[1:0] == hur_offset[3:2]) ?
                                                 refill_word :
                                                 select_word(refill_line_next, hur_offset[3:2]);
                            data_ok           <= 1'b1;
                            hur_valid         <= 1'b0;
                            hur_probe_pending <= 1'b0;
                            hur_second_miss   <= 1'b0;
                            state             <= S_IDLE;
`ifndef SYNTHESIS
                            hur_same_line_hit_cnt <= hur_same_line_hit_cnt + 32'd1;
`endif
                        end
                        else if (HIT_UNDER_REFILL && hur_valid &&
                                 (hur_second_miss ||
                                  (hur_probe_pending && !hur_cache_hit))) begin
                            req_op          <= 1'b0;
                            req_index       <= hur_index;
                            req_tag         <= hur_tag;
                            req_offset      <= hur_offset;
                            req_wstrb       <= 4'b0;
                            req_wdata       <= 32'b0;
                            req_dual_read   <= 1'b0;
                            hur_valid         <= 1'b0;
                            hur_probe_pending <= 1'b0;
                            hur_second_miss   <= 1'b0;
                            hur_replay_setup  <= 1'b1;
                            hur_replay_active <= 1'b0;
                            state             <= S_IDLE;
`ifndef SYNTHESIS
                            hur_replay_setup_cnt <= hur_replay_setup_cnt + 32'd1;
                            if (hur_debug_event_seq < 32'd16) begin
                                $display("HUR_EVENT|kind=replay_setup|seq=%0d|cycle=%0d|addr=0x%08x|installed_line=0x%08x",
                                         hur_debug_event_seq, hur_debug_cycle_cnt,
                                         compose_byte_addr(hur_tag, hur_index, hur_offset),
                                         compose_line_addr(req_tag, req_index));
                                hur_debug_event_seq <= hur_debug_event_seq + 32'd1;
                            end
`endif
                        end
                        else begin
                            hur_replay_setup <= 1'b0;
                            state <= S_IDLE;
                        end
                    end
                end
            end

            S_CACOP: begin
                if (cacop_need_wb) begin
                    repl_way       <= cacop_target_way;
                    repl_old_tag   <= cacop_tgt_tag;
                    repl_old_data  <= cacop_tgt_line;
                    state           <= S_CACOP_WB;
                end
                else begin
                    cacop_data_ok <= 1'b1;
                    state         <= S_IDLE;
                end
            end

            S_CACOP_WB: begin
                if (wr_rdy) begin
                    cacop_data_ok <= 1'b1;
                    state         <= S_IDLE;
                end
            end

            default: state <= S_INIT;
        endcase
    end
end

endmodule

// M18.4 1024-deep inferred block RAMs. A 1024 x 32 byte-write memory maps to
// one RAMB36; the tag memory is small enough for one RAMB18 per way.
module m184_tagv_sram_1024 (
    input  wire        clka,
    input  wire        ena,
    input  wire [0:0]  wea,
    input  wire [9:0]  addra,
    input  wire [20:0] dina,
    output reg  [20:0] douta
);
    (* ram_style = "block" *) reg [20:0] mem [0:1023];
    always @(posedge clka) begin
        if (ena) begin
            douta <= mem[addra];
            if (wea[0])
                mem[addra] <= dina;
        end
    end
endmodule

module m184_data_bank_sram_1024 (
    input  wire        clka,
    input  wire        ena,
    input  wire [3:0]  wea,
    input  wire [9:0]  addra,
    input  wire [31:0] dina,
    output reg  [31:0] douta
);
    (* ram_style = "block" *) reg [31:0] mem [0:1023];
    always @(posedge clka) begin
        if (ena) begin
            douta <= mem[addra];
            if (wea[0]) mem[addra][ 7: 0] <= dina[ 7: 0];
            if (wea[1]) mem[addra][15: 8] <= dina[15: 8];
            if (wea[2]) mem[addra][23:16] <= dina[23:16];
            if (wea[3]) mem[addra][31:24] <= dina[31:24];
        end
    end
endmodule

// ================================================================
// M17.5D 512-deep inferred block RAMs.
// Synchronous read-first behavior matches the existing Xilinx cache RAM IP.
// Vivado should map each instance to one RAMB18 at 512 x 21 / 512 x 32.
// ================================================================
module m175d_tagv_sram_512 (
    input  wire        clka,
    input  wire        ena,
    input  wire [0:0]  wea,
    input  wire [8:0]  addra,
    input  wire [20:0] dina,
    output reg  [20:0] douta
);
    (* ram_style = "block" *) reg [20:0] mem [0:511];
    always @(posedge clka) begin
        if (ena) begin
            douta <= mem[addra];
            if (wea[0])
                mem[addra] <= dina;
        end
    end
endmodule

module m175d_data_bank_sram_512 (
    input  wire        clka,
    input  wire        ena,
    input  wire [3:0]  wea,
    input  wire [8:0]  addra,
    input  wire [31:0] dina,
    output reg  [31:0] douta
);
    (* ram_style = "block" *) reg [31:0] mem [0:511];
    always @(posedge clka) begin
        if (ena) begin
            douta <= mem[addra];
            if (wea[0]) mem[addra][ 7: 0] <= dina[ 7: 0];
            if (wea[1]) mem[addra][15: 8] <= dina[15: 8];
            if (wea[2]) mem[addra][23:16] <= dina[23:16];
            if (wea[3]) mem[addra][31:24] <= dina[31:24];
        end
    end
endmodule
