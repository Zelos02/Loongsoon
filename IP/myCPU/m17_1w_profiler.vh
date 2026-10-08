// =============================================================================
// M17.5D simulation-only wide profiler (based on M17.1W/M17.3F)
// =============================================================================
// This file is included inside mycpu_top while ENABLE_SIM_PERF_DIAG is active.
// It never enters the SYNTHESIS build.  Every adjacent pair of SoC timer reads
// defines one candidate segment.  Programs that read both SoC and CPU counters
// therefore emit several segments; analyze_m17_5d.py automatically selects the
// segment with the largest cycle count.  M17.5D also snapshots exact DCache
// lookup/miss/victim counters so 32KB vs 16KB A/B can be measured directly.
// =============================================================================

localparam integer WPROF_MIX_CLASSES       = 18;
localparam integer WPROF_PAIR_CLASSES      = 6;
localparam integer WPROF_BRANCH_SLOTS      = 128;
localparam integer WPROF_MEMPC_SLOTS       = 64;
localparam integer WPROF_SEEN_SLOTS        = 2048;
localparam integer WPROF_RECENT_LINES      = 1024;
localparam integer WPROF_LINEUSE_SLOTS     = 1024;

localparam [4:0] WPROF_MIX_ALU_REG         = 5'd0;
localparam [4:0] WPROF_MIX_ALU_IMM         = 5'd1;
localparam [4:0] WPROF_MIX_SHIFT           = 5'd2;
localparam [4:0] WPROF_MIX_LOAD_BYTE       = 5'd3;
localparam [4:0] WPROF_MIX_LOAD_HALF       = 5'd4;
localparam [4:0] WPROF_MIX_LOAD_WORD       = 5'd5;
localparam [4:0] WPROF_MIX_STORE_BYTE      = 5'd6;
localparam [4:0] WPROF_MIX_STORE_HALF      = 5'd7;
localparam [4:0] WPROF_MIX_STORE_WORD      = 5'd8;
localparam [4:0] WPROF_MIX_COND_BRANCH     = 5'd9;
localparam [4:0] WPROF_MIX_DIRECT_JUMP     = 5'd10;
localparam [4:0] WPROF_MIX_INDIRECT_JUMP   = 5'd11;
localparam [4:0] WPROF_MIX_CALL            = 5'd12;
localparam [4:0] WPROF_MIX_RETURN          = 5'd13;
localparam [4:0] WPROF_MIX_MULTIPLY        = 5'd14;
localparam [4:0] WPROF_MIX_DIVIDE          = 5'd15;
localparam [4:0] WPROF_MIX_SYSTEM          = 5'd16;
localparam [4:0] WPROF_MIX_OTHER           = 5'd17;

function [4:0] wprof_inst_class;
    input [31:0] inst;
    reg [5:0] op6;
    reg [3:0] op4;
    reg [1:0] op2;
    reg [4:0] op5;
    reg [4:0] rd_f;
    reg [4:0] rj_f;
    reg [15:0] i16_f;
    begin
        op6  = inst[31:26];
        op4  = inst[25:22];
        op2  = inst[21:20];
        op5  = inst[19:15];
        rd_f = inst[4:0];
        rj_f = inst[9:5];
        i16_f = inst[25:10];

        // ABI-standard calls and returns are separated from generic jumps.
        if ((op6 == 6'h15) || ((op6 == 6'h13) && (rd_f == 5'd1)))
            wprof_inst_class = WPROF_MIX_CALL;
        else if ((op6 == 6'h13) && (rd_f == 5'd0) &&
                 (rj_f == 5'd1) && (i16_f == 16'b0))
            wprof_inst_class = WPROF_MIX_RETURN;
        else if ((op6 >= 6'h16) && (op6 <= 6'h1b))
            wprof_inst_class = WPROF_MIX_COND_BRANCH;
        else if (op6 == 6'h14)
            wprof_inst_class = WPROF_MIX_DIRECT_JUMP;
        else if (op6 == 6'h13)
            wprof_inst_class = WPROF_MIX_INDIRECT_JUMP;
        else if ((op6 == 6'h00) && (op4 == 4'h0) &&
                 (op2 == 2'h1) && (op5 >= 5'h18) && (op5 <= 5'h1a))
            wprof_inst_class = WPROF_MIX_MULTIPLY;
        else if ((op6 == 6'h00) && (op4 == 4'h0) &&
                 (op2 == 2'h2) && (op5 <= 5'h03))
            wprof_inst_class = WPROF_MIX_DIVIDE;
        else if ((op6 == 6'h0a) && ((op4 == 4'h0) || (op4 == 4'h8)))
            wprof_inst_class = WPROF_MIX_LOAD_BYTE;
        else if ((op6 == 6'h0a) && ((op4 == 4'h1) || (op4 == 4'h9)))
            wprof_inst_class = WPROF_MIX_LOAD_HALF;
        else if ((op6 == 6'h0a) && (op4 == 4'h2))
            wprof_inst_class = WPROF_MIX_LOAD_WORD;
        else if ((op6 == 6'h0a) && (op4 == 4'h4))
            wprof_inst_class = WPROF_MIX_STORE_BYTE;
        else if ((op6 == 6'h0a) && (op4 == 4'h5))
            wprof_inst_class = WPROF_MIX_STORE_HALF;
        else if ((op6 == 6'h0a) && (op4 == 4'h6))
            wprof_inst_class = WPROF_MIX_STORE_WORD;
        else if (((op6 == 6'h00) && (op4 == 4'h1) && (op2 == 2'h0) &&
                  ((op5 == 5'h01) || (op5 == 5'h09) || (op5 == 5'h11))) ||
                 ((op6 == 6'h00) && (op4 == 4'h0) && (op2 == 2'h1) &&
                  ((op5 == 5'h0e) || (op5 == 5'h0f) || (op5 == 5'h10))))
            wprof_inst_class = WPROF_MIX_SHIFT;
        else if ((op6 == 6'h00) &&
                 ((op4 == 4'h8) || (op4 == 4'h9) || (op4 == 4'ha) ||
                  (op4 == 4'hd) || (op4 == 4'he) || (op4 == 4'hf)))
            wprof_inst_class = WPROF_MIX_ALU_IMM;
        else if (((op6 == 6'h00) && (op4 == 4'h0) && (op2 == 2'h1)) ||
                 (op6 == 6'h05) || (op6 == 6'h07))
            wprof_inst_class = WPROF_MIX_ALU_REG;
        else if ((op6 == 6'h01) || (inst == 32'h06483800) ||
                 (inst == 32'h06488000) || (inst[31:24] == 8'h20) ||
                 (inst[31:24] == 8'h21) ||
                 ((op6 == 6'h00) && (op4 == 4'h0) && (op2 == 2'h0)))
            wprof_inst_class = WPROF_MIX_SYSTEM;
        else
            wprof_inst_class = WPROF_MIX_OTHER;
    end
endfunction

function [2:0] wprof_pair_class;
    input [31:0] inst;
    reg [4:0] c;
    begin
        c = wprof_inst_class(inst);
        if ((c == WPROF_MIX_ALU_REG) || (c == WPROF_MIX_ALU_IMM) ||
            (c == WPROF_MIX_SHIFT))
            wprof_pair_class = 3'd0; // ALU
        else if ((c == WPROF_MIX_LOAD_BYTE) || (c == WPROF_MIX_LOAD_HALF) ||
                 (c == WPROF_MIX_LOAD_WORD))
            wprof_pair_class = 3'd1; // load
        else if ((c == WPROF_MIX_STORE_BYTE) || (c == WPROF_MIX_STORE_HALF) ||
                 (c == WPROF_MIX_STORE_WORD))
            wprof_pair_class = 3'd2; // store
        else if ((c == WPROF_MIX_COND_BRANCH) || (c == WPROF_MIX_DIRECT_JUMP) ||
                 (c == WPROF_MIX_INDIRECT_JUMP) || (c == WPROF_MIX_CALL) ||
                 (c == WPROF_MIX_RETURN))
            wprof_pair_class = 3'd3; // branch/control
        else if (c == WPROF_MIX_MULTIPLY)
            wprof_pair_class = 3'd4; // multiply
        else
            wprof_pair_class = 3'd5; // divide/system/other
    end
endfunction

function [6:0] wprof_branch_index;
    input [31:0] pc;
    begin
        wprof_branch_index = pc[8:2] ^ pc[15:9] ^ pc[22:16];
    end
endfunction

function [5:0] wprof_mempc_index;
    input [31:0] pc;
    begin
        wprof_mempc_index = pc[7:2] ^ pc[13:8] ^ pc[19:14];
    end
endfunction

function [10:0] wprof_seen_index;
    input [27:0] line;
    begin
        wprof_seen_index = line[10:0] ^ line[21:11] ^ {5'b0,line[27:22]};
    end
endfunction

function [9:0] wprof_lineuse_index;
    input [27:0] line;
    begin
        wprof_lineuse_index = line[9:0] ^ line[19:10] ^ {2'b0,line[27:20]};
    end
endfunction

function [2:0] wprof_popcount4;
    input [3:0] value;
    begin
        wprof_popcount4 = value[0] + value[1] + value[2] + value[3];
    end
endfunction

wire [4:0] wprof_main_mix_class = wprof_inst_class(u_mycpu_core.id_stage.ds_inst);
wire [4:0] wprof_young_mix_class = wprof_inst_class(u_mycpu_core.id_stage.slot1_candidate_inst);
wire [2:0] wprof_old_pair_class = wprof_pair_class(u_mycpu_core.id_stage.ds_inst);
wire [2:0] wprof_young_pair_class = wprof_pair_class(u_mycpu_core.id_stage.slot1_candidate_inst);
wire [31:0] wprof_dcache_addr = DCACHE_SETS_512 ?
                                   {u_dcache.req_tag[18:0], u_dcache.req_index,
                                    u_dcache.req_offset} :
                                   {u_dcache.req_tag, u_dcache.req_index[7:0],
                                    u_dcache.req_offset};
wire [27:0] wprof_dcache_line = DCACHE_SETS_512 ?
                                {u_dcache.req_tag[18:0], u_dcache.req_index} :
                                {u_dcache.req_tag, u_dcache.req_index[7:0]};
wire [3:0] wprof_dcache_word_bit = (4'b0001 << u_dcache.req_offset[3:2]);
wire [6:0] wprof_branch_idx = wprof_branch_index(diag_branch_pc);
wire [5:0] wprof_mempc_idx = wprof_mempc_index(diag_mem_wait_pc);
wire [8:0] wprof_actual_bht_idx = diag_branch_pc[10:2] ^ u_mycpu_core.u_branch_predictor.ghr;
wire wprof_branch_direction_miss = u_mycpu_core.id_stage.bp_direction_miss;
wire wprof_branch_target_miss = u_mycpu_core.id_stage.bp_target_miss;
wire [2:0] wprof_branch_type = u_mycpu_core.id_stage.bp_update_is_return ? 3'd3 :
                               u_mycpu_core.id_stage.bp_update_is_call ? 3'd2 :
                               u_mycpu_core.id_stage.bp_update_is_indirect ? 3'd4 :
                               u_mycpu_core.id_stage.bp_update_is_cond ? 3'd0 : 3'd1;

reg        wprof_active;
reg [31:0] wprof_timer_read_count;
reg [31:0] wprof_segment_index;
reg [31:0] wprof_cycles;
reg [31:0] wprof_issue_cycles;
reg [31:0] wprof_issued_insts;
reg [31:0] wprof_dual_cycles;
reg [31:0] wprof_redirects;
reg [31:0] wprof_mix [0:WPROF_MIX_CLASSES-1];
reg [31:0] wprof_pair_matrix [0:(WPROF_PAIR_CLASSES*WPROF_PAIR_CLASSES)-1];
reg [31:0] wprof_pair_fail_raw;
reg [31:0] wprof_pair_fail_waw;
reg [31:0] wprof_pair_fail_structural;
reg [31:0] wprof_pair_fail_unsupported;
reg [31:0] wprof_pair_fail_older_not_ready;
reg [31:0] wprof_pair_fail_boundary;

// Mutually exclusive CPI stack.
reg [31:0] wprof_cpi_productive;
reg [31:0] wprof_cpi_frontend_empty;
reg [31:0] wprof_cpi_icache_wait;
reg [31:0] wprof_cpi_branch_recovery;
reg [31:0] wprof_cpi_decode_dependency;
reg [31:0] wprof_cpi_load_use;
reg [31:0] wprof_cpi_mul_wait;
reg [31:0] wprof_cpi_div_wait;
reg [31:0] wprof_cpi_dcache_lookup;
reg [31:0] wprof_cpi_dcache_rdreq;
reg [31:0] wprof_cpi_dcache_refill;
reg [31:0] wprof_cpi_dcache_wb;
reg [31:0] wprof_cpi_uncached;
reg [31:0] wprof_cpi_backend_structural;
reg [31:0] wprof_cpi_other;

// 128-entry tagged branch profiler.
reg        wprof_br_valid [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_pc [0:WPROF_BRANCH_SLOTS-1];
reg [2:0]  wprof_br_type [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_exec [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_taken [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_miss [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_dir_miss [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_tgt_miss [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_forward [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_backward [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_flips [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_br_alias [0:WPROF_BRANCH_SLOTS-1];
reg [8:0]  wprof_br_bht_index [0:WPROF_BRANCH_SLOTS-1];
reg        wprof_br_last_taken [0:WPROF_BRANCH_SLOTS-1];
reg [15:0] wprof_br_run_len [0:WPROF_BRANCH_SLOTS-1];
reg [15:0] wprof_br_max_taken_run [0:WPROF_BRANCH_SLOTS-1];
reg [15:0] wprof_br_max_nt_run [0:WPROF_BRANCH_SLOTS-1];
reg [31:0] wprof_branch_alias_events;
reg [31:0] wprof_call_count;
reg [31:0] wprof_return_count;
reg [31:0] wprof_indirect_count;
reg [31:0] wprof_return_target_miss;
reg [31:0] wprof_indirect_target_miss;

// Memory miss taxonomy and per-PC stride profiler.
reg [31:0] wprof_miss_compulsory;
reg [31:0] wprof_miss_conflict;
reg [31:0] wprof_miss_capacity;
reg [31:0] wprof_miss_seen_alias;
reg        wprof_seen_valid [0:WPROF_SEEN_SLOTS-1];
reg [27:0] wprof_seen_line [0:WPROF_SEEN_SLOTS-1];
reg        wprof_recent_valid [0:WPROF_RECENT_LINES-1];
reg [27:0] wprof_recent_line [0:WPROF_RECENT_LINES-1];
reg [9:0]  wprof_recent_ptr;

reg        wprof_mempc_valid [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_pc [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_access [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_load [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_store [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_miss [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_same_line [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_next_line [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_const_stride [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_alias [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_last_addr [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_last_stride [0:WPROF_MEMPC_SLOTS-1];
reg        wprof_mempc_have_last [0:WPROF_MEMPC_SLOTS-1];
reg [31:0] wprof_mempc_alias_events;

// Approximate line-use table.  Hash replacement is reported explicitly.
reg        wprof_lineuse_valid [0:WPROF_LINEUSE_SLOTS-1];
reg [27:0] wprof_lineuse_line [0:WPROF_LINEUSE_SLOTS-1];
reg [3:0]  wprof_lineuse_mask [0:WPROF_LINEUSE_SLOTS-1];
reg [31:0] wprof_lineuse_1word;
reg [31:0] wprof_lineuse_2word;
reg [31:0] wprof_lineuse_3word;
reg [31:0] wprof_lineuse_4word;
reg [31:0] wprof_lineuse_hash_evictions;

// Potential overlap available to a future single-MSHR/hit-under-refill design.
reg [31:0] wprof_nb_refill_cycles;
reg [31:0] wprof_nb_independent_alu;
reg [31:0] wprof_nb_independent_branch;
reg [31:0] wprof_nb_memory_candidate;
reg [31:0] wprof_nb_same_line_candidate;
reg [31:0] wprof_nb_load_use_dependency;
reg [31:0] wprof_nb_ifq_buffered_cycles;
reg [31:0] wprof_nb_ifq_entries_sum;

// Segment baselines for the hardware HUR counters, which are maintained inside
// DCache from reset.  Subtracting these snapshots keeps each timer segment
// independent even for tests with several timer reads.
reg [31:0] wprof_hur_accept_base;
reg [31:0] wprof_hur_same_line_base;
reg [31:0] wprof_hur_other_line_base;
reg [31:0] wprof_hur_second_miss_base;
reg [31:0] wprof_hur_replay_setup_base;
reg [31:0] wprof_hur_replay_hit_base;
reg [31:0] wprof_hur_replay_miss_base;
reg [31:0] wprof_hur_word_wait_base;
reg [31:0] wprof_hur_store_block_base;

// Segment baselines for the generic simulation-only ICache counters.
reg [31:0] wprof_ic_lookup_base;
reg [31:0] wprof_ic_hit_base;
reg [31:0] wprof_ic_miss_base;
reg [31:0] wprof_ic_refill_line_base;
reg [31:0] wprof_ic_rdreq_cycle_base;
reg [31:0] wprof_ic_refill_cycle_base;
reg [31:0] wprof_ic_way0_hit_base;
reg [31:0] wprof_ic_way1_hit_base;
reg [31:0] wprof_ic_way2_hit_base;
reg [31:0] wprof_ic_way3_hit_base;
reg [31:0] wprof_ic_way0_victim_base;
reg [31:0] wprof_ic_way1_victim_base;
reg [31:0] wprof_ic_way2_victim_base;
reg [31:0] wprof_ic_way3_victim_base;

// Segment baselines for exact DCache counters.  These complement the older
// approximate hash-based miss classifier and are the primary M17.5D A/B data.
reg [31:0] wprof_dc_lookup_base;
reg [31:0] wprof_dc_hit_base;
reg [31:0] wprof_dc_miss_base;
reg [31:0] wprof_dc_refill_line_base;
reg [31:0] wprof_dc_rdreq_cycle_base;
reg [31:0] wprof_dc_refill_cycle_base;
reg [31:0] wprof_dc_way0_hit_base;
reg [31:0] wprof_dc_way1_hit_base;
reg [31:0] wprof_dc_way2_hit_base;
reg [31:0] wprof_dc_way3_hit_base;
reg [31:0] wprof_dc_way0_victim_base;
reg [31:0] wprof_dc_way1_victim_base;
reg [31:0] wprof_dc_way2_victim_base;
reg [31:0] wprof_dc_way3_victim_base;

integer wprof_i;
integer wprof_j;
integer wprof_matrix_idx;
integer wprof_recent_found;
integer wprof_tmp_1word;
integer wprof_tmp_2word;
integer wprof_tmp_3word;
integer wprof_tmp_4word;
reg [31:0] wprof_stride_now;
reg [2:0] wprof_words_used;

always @(posedge clk) begin
    if (!resetn) begin
        wprof_active <= 1'b0;
        wprof_timer_read_count <= 32'b0;
        wprof_segment_index <= 32'b0;
        wprof_cycles <= 32'b0;
        wprof_issue_cycles <= 32'b0;
        wprof_issued_insts <= 32'b0;
        wprof_dual_cycles <= 32'b0;
        wprof_redirects <= 32'b0;
        wprof_pair_fail_raw <= 32'b0;
        wprof_pair_fail_waw <= 32'b0;
        wprof_pair_fail_structural <= 32'b0;
        wprof_pair_fail_unsupported <= 32'b0;
        wprof_pair_fail_older_not_ready <= 32'b0;
        wprof_pair_fail_boundary <= 32'b0;
        wprof_cpi_productive <= 32'b0;
        wprof_cpi_frontend_empty <= 32'b0;
        wprof_cpi_icache_wait <= 32'b0;
        wprof_cpi_branch_recovery <= 32'b0;
        wprof_cpi_decode_dependency <= 32'b0;
        wprof_cpi_load_use <= 32'b0;
        wprof_cpi_mul_wait <= 32'b0;
        wprof_cpi_div_wait <= 32'b0;
        wprof_cpi_dcache_lookup <= 32'b0;
        wprof_cpi_dcache_rdreq <= 32'b0;
        wprof_cpi_dcache_refill <= 32'b0;
        wprof_cpi_dcache_wb <= 32'b0;
        wprof_cpi_uncached <= 32'b0;
        wprof_cpi_backend_structural <= 32'b0;
        wprof_cpi_other <= 32'b0;
        wprof_branch_alias_events <= 32'b0;
        wprof_call_count <= 32'b0;
        wprof_return_count <= 32'b0;
        wprof_indirect_count <= 32'b0;
        wprof_return_target_miss <= 32'b0;
        wprof_indirect_target_miss <= 32'b0;
        wprof_miss_compulsory <= 32'b0;
        wprof_miss_conflict <= 32'b0;
        wprof_miss_capacity <= 32'b0;
        wprof_miss_seen_alias <= 32'b0;
        wprof_recent_ptr <= 10'b0;
        wprof_mempc_alias_events <= 32'b0;
        wprof_lineuse_1word <= 32'b0;
        wprof_lineuse_2word <= 32'b0;
        wprof_lineuse_3word <= 32'b0;
        wprof_lineuse_4word <= 32'b0;
        wprof_lineuse_hash_evictions <= 32'b0;
        wprof_nb_refill_cycles <= 32'b0;
        wprof_nb_independent_alu <= 32'b0;
        wprof_nb_independent_branch <= 32'b0;
        wprof_nb_memory_candidate <= 32'b0;
        wprof_nb_same_line_candidate <= 32'b0;
        wprof_nb_load_use_dependency <= 32'b0;
        wprof_nb_ifq_buffered_cycles <= 32'b0;
        wprof_nb_ifq_entries_sum <= 32'b0;
        wprof_hur_accept_base <= 32'b0;
        wprof_hur_same_line_base <= 32'b0;
        wprof_hur_other_line_base <= 32'b0;
        wprof_hur_second_miss_base <= 32'b0;
        wprof_hur_replay_setup_base <= 32'b0;
        wprof_hur_replay_hit_base <= 32'b0;
        wprof_hur_replay_miss_base <= 32'b0;
        wprof_hur_word_wait_base <= 32'b0;
        wprof_hur_store_block_base <= 32'b0;
        wprof_ic_lookup_base <= 32'b0;
        wprof_ic_hit_base <= 32'b0;
        wprof_ic_miss_base <= 32'b0;
        wprof_ic_refill_line_base <= 32'b0;
        wprof_ic_rdreq_cycle_base <= 32'b0;
        wprof_ic_refill_cycle_base <= 32'b0;
        wprof_ic_way0_hit_base <= 32'b0;
        wprof_ic_way1_hit_base <= 32'b0;
        wprof_ic_way2_hit_base <= 32'b0;
        wprof_ic_way3_hit_base <= 32'b0;
        wprof_ic_way0_victim_base <= 32'b0;
        wprof_ic_way1_victim_base <= 32'b0;
        wprof_ic_way2_victim_base <= 32'b0;
        wprof_ic_way3_victim_base <= 32'b0;
        wprof_dc_lookup_base <= 32'b0;
        wprof_dc_hit_base <= 32'b0;
        wprof_dc_miss_base <= 32'b0;
        wprof_dc_refill_line_base <= 32'b0;
        wprof_dc_rdreq_cycle_base <= 32'b0;
        wprof_dc_refill_cycle_base <= 32'b0;
        wprof_dc_way0_hit_base <= 32'b0;
        wprof_dc_way1_hit_base <= 32'b0;
        wprof_dc_way2_hit_base <= 32'b0;
        wprof_dc_way3_hit_base <= 32'b0;
        wprof_dc_way0_victim_base <= 32'b0;
        wprof_dc_way1_victim_base <= 32'b0;
        wprof_dc_way2_victim_base <= 32'b0;
        wprof_dc_way3_victim_base <= 32'b0;
        for (wprof_i = 0; wprof_i < WPROF_MIX_CLASSES; wprof_i = wprof_i + 1)
            wprof_mix[wprof_i] <= 32'b0;
        for (wprof_i = 0; wprof_i < WPROF_PAIR_CLASSES*WPROF_PAIR_CLASSES; wprof_i = wprof_i + 1)
            wprof_pair_matrix[wprof_i] <= 32'b0;
        for (wprof_i = 0; wprof_i < WPROF_BRANCH_SLOTS; wprof_i = wprof_i + 1) begin
            wprof_br_valid[wprof_i] <= 1'b0;
            wprof_br_pc[wprof_i] <= 32'b0;
            wprof_br_type[wprof_i] <= 3'b0;
            wprof_br_exec[wprof_i] <= 32'b0;
            wprof_br_taken[wprof_i] <= 32'b0;
            wprof_br_miss[wprof_i] <= 32'b0;
            wprof_br_dir_miss[wprof_i] <= 32'b0;
            wprof_br_tgt_miss[wprof_i] <= 32'b0;
            wprof_br_forward[wprof_i] <= 32'b0;
            wprof_br_backward[wprof_i] <= 32'b0;
            wprof_br_flips[wprof_i] <= 32'b0;
            wprof_br_alias[wprof_i] <= 32'b0;
            wprof_br_bht_index[wprof_i] <= 9'b0;
            wprof_br_last_taken[wprof_i] <= 1'b0;
            wprof_br_run_len[wprof_i] <= 16'b0;
            wprof_br_max_taken_run[wprof_i] <= 16'b0;
            wprof_br_max_nt_run[wprof_i] <= 16'b0;
        end
        for (wprof_i = 0; wprof_i < WPROF_MEMPC_SLOTS; wprof_i = wprof_i + 1) begin
            wprof_mempc_valid[wprof_i] <= 1'b0;
            wprof_mempc_pc[wprof_i] <= 32'b0;
            wprof_mempc_access[wprof_i] <= 32'b0;
            wprof_mempc_load[wprof_i] <= 32'b0;
            wprof_mempc_store[wprof_i] <= 32'b0;
            wprof_mempc_miss[wprof_i] <= 32'b0;
            wprof_mempc_same_line[wprof_i] <= 32'b0;
            wprof_mempc_next_line[wprof_i] <= 32'b0;
            wprof_mempc_const_stride[wprof_i] <= 32'b0;
            wprof_mempc_alias[wprof_i] <= 32'b0;
            wprof_mempc_last_addr[wprof_i] <= 32'b0;
            wprof_mempc_last_stride[wprof_i] <= 32'b0;
            wprof_mempc_have_last[wprof_i] <= 1'b0;
        end
        for (wprof_i = 0; wprof_i < WPROF_SEEN_SLOTS; wprof_i = wprof_i + 1) begin
            wprof_seen_valid[wprof_i] <= 1'b0;
            wprof_seen_line[wprof_i] <= 28'b0;
        end
        for (wprof_i = 0; wprof_i < WPROF_RECENT_LINES; wprof_i = wprof_i + 1) begin
            wprof_recent_valid[wprof_i] <= 1'b0;
            wprof_recent_line[wprof_i] <= 28'b0;
        end
        for (wprof_i = 0; wprof_i < WPROF_LINEUSE_SLOTS; wprof_i = wprof_i + 1) begin
            wprof_lineuse_valid[wprof_i] <= 1'b0;
            wprof_lineuse_line[wprof_i] <= 28'b0;
            wprof_lineuse_mask[wprof_i] <= 4'b0;
        end
    end else if (perf_timer_read_done) begin
        // Print the segment that just ended.  All values are pre-NBA values,
        // so this display occurs before the segment counters are cleared.
        wprof_timer_read_count <= wprof_timer_read_count + 32'd1;
        if (wprof_active) begin
            wprof_tmp_1word = 0;
            wprof_tmp_2word = 0;
            wprof_tmp_3word = 0;
            wprof_tmp_4word = 0;
            for (wprof_i = 0; wprof_i < WPROF_LINEUSE_SLOTS; wprof_i = wprof_i + 1) begin
                if (wprof_lineuse_valid[wprof_i]) begin
                    wprof_words_used = wprof_popcount4(wprof_lineuse_mask[wprof_i]);
                    if (wprof_words_used == 3'd1) wprof_tmp_1word = wprof_tmp_1word + 1;
                    else if (wprof_words_used == 3'd2) wprof_tmp_2word = wprof_tmp_2word + 1;
                    else if (wprof_words_used == 3'd3) wprof_tmp_3word = wprof_tmp_3word + 1;
                    else if (wprof_words_used == 3'd4) wprof_tmp_4word = wprof_tmp_4word + 1;
                end
            end
            $display("PERF_DIAG_BEGIN|version=M17.5D-D12|segment=%0d", wprof_segment_index);
            $display("PERF_DIAG|window|segment=%0d|timer_read_end=%0d|cycles=%0d|selection=choose_max_cycles|valid=%0d",
                     wprof_segment_index, wprof_timer_read_count + 32'd1,
                     wprof_cycles, (wprof_cycles != 0));
            $display("PERF_DIAG|summary|cycles=%0d|issue_cycles=%0d|issued_insts=%0d|ipc_x1000=%0d|dual_cycles=%0d|redirects=%0d",
                     wprof_cycles, wprof_issue_cycles, wprof_issued_insts,
                     diag_ratio_x1000(wprof_issued_insts, wprof_cycles),
                     wprof_dual_cycles, wprof_redirects);
            $display("PERF_DIAG|instruction_mix|alu_reg=%0d|alu_imm=%0d|shift=%0d|load_byte=%0d|load_half=%0d|load_word=%0d|store_byte=%0d|store_half=%0d|store_word=%0d|conditional_branch=%0d|direct_jump=%0d|indirect_jump=%0d|call=%0d|return=%0d|multiply=%0d|divide=%0d|system=%0d|other=%0d",
                     wprof_mix[0],wprof_mix[1],wprof_mix[2],wprof_mix[3],wprof_mix[4],wprof_mix[5],
                     wprof_mix[6],wprof_mix[7],wprof_mix[8],wprof_mix[9],wprof_mix[10],wprof_mix[11],
                     wprof_mix[12],wprof_mix[13],wprof_mix[14],wprof_mix[15],wprof_mix[16],wprof_mix[17]);
            $display("PERF_DIAG|cpi_stack|productive=%0d|frontend_empty=%0d|icache_wait=%0d|branch_recovery=%0d|decode_dependency=%0d|load_use=%0d|mul_wait=%0d|div_wait=%0d|dcache_lookup=%0d|dcache_rdreq=%0d|dcache_refill=%0d|dcache_writeback=%0d|uncached_wait=%0d|backend_structural=%0d|other=%0d|sum=%0d",
                     wprof_cpi_productive,wprof_cpi_frontend_empty,wprof_cpi_icache_wait,
                     wprof_cpi_branch_recovery,wprof_cpi_decode_dependency,wprof_cpi_load_use,
                     wprof_cpi_mul_wait,wprof_cpi_div_wait,wprof_cpi_dcache_lookup,
                     wprof_cpi_dcache_rdreq,wprof_cpi_dcache_refill,wprof_cpi_dcache_wb,
                     wprof_cpi_uncached,wprof_cpi_backend_structural,wprof_cpi_other,
                     wprof_cpi_productive+wprof_cpi_frontend_empty+wprof_cpi_icache_wait+
                     wprof_cpi_branch_recovery+wprof_cpi_decode_dependency+wprof_cpi_load_use+
                     wprof_cpi_mul_wait+wprof_cpi_div_wait+wprof_cpi_dcache_lookup+
                     wprof_cpi_dcache_rdreq+wprof_cpi_dcache_refill+wprof_cpi_dcache_wb+
                     wprof_cpi_uncached+wprof_cpi_backend_structural+wprof_cpi_other);
            $display("PERF_DIAG|pair_fail|raw=%0d|waw=%0d|structural=%0d|unsupported=%0d|older_not_ready=%0d|boundary=%0d",
                     wprof_pair_fail_raw,wprof_pair_fail_waw,wprof_pair_fail_structural,
                     wprof_pair_fail_unsupported,wprof_pair_fail_older_not_ready,wprof_pair_fail_boundary);
            for (wprof_i = 0; wprof_i < WPROF_PAIR_CLASSES; wprof_i = wprof_i + 1)
                $display("PERF_DIAG|pair_matrix|old=%0d|alu=%0d|load=%0d|store=%0d|branch=%0d|mul=%0d|other=%0d",
                         wprof_i,
                         wprof_pair_matrix[wprof_i*WPROF_PAIR_CLASSES+0],
                         wprof_pair_matrix[wprof_i*WPROF_PAIR_CLASSES+1],
                         wprof_pair_matrix[wprof_i*WPROF_PAIR_CLASSES+2],
                         wprof_pair_matrix[wprof_i*WPROF_PAIR_CLASSES+3],
                         wprof_pair_matrix[wprof_i*WPROF_PAIR_CLASSES+4],
                         wprof_pair_matrix[wprof_i*WPROF_PAIR_CLASSES+5]);
            $display("PERF_DIAG|branch_summary|calls=%0d|returns=%0d|indirect=%0d|return_target_miss=%0d|indirect_target_miss=%0d|table_alias_events=%0d",
                     wprof_call_count,wprof_return_count,wprof_indirect_count,
                     wprof_return_target_miss,wprof_indirect_target_miss,wprof_branch_alias_events);
            for (wprof_i = 0; wprof_i < WPROF_BRANCH_SLOTS; wprof_i = wprof_i + 1) begin
                if (wprof_br_valid[wprof_i] && (wprof_br_exec[wprof_i] != 0))
                    $display("PERF_DIAG|branch_pc|slot=%0d|pc=0x%08x|type=%0d|exec=%0d|taken=%0d|miss=%0d|direction_miss=%0d|target_miss=%0d|forward=%0d|backward=%0d|flips=%0d|max_taken_run=%0d|max_nt_run=%0d|bht_index=%0d|alias=%0d|accuracy_x10000=%0d",
                             wprof_i,wprof_br_pc[wprof_i],wprof_br_type[wprof_i],
                             wprof_br_exec[wprof_i],wprof_br_taken[wprof_i],wprof_br_miss[wprof_i],
                             wprof_br_dir_miss[wprof_i],wprof_br_tgt_miss[wprof_i],
                             wprof_br_forward[wprof_i],wprof_br_backward[wprof_i],wprof_br_flips[wprof_i],
                             wprof_br_max_taken_run[wprof_i],wprof_br_max_nt_run[wprof_i],
                             wprof_br_bht_index[wprof_i],wprof_br_alias[wprof_i],
                             64'd10000-diag_ratio_x10000(wprof_br_miss[wprof_i],wprof_br_exec[wprof_i]));
            end
            $display("PERF_DIAG|miss_type|approx=1|compulsory=%0d|conflict=%0d|capacity=%0d|seen_hash_alias=%0d",
                     wprof_miss_compulsory,wprof_miss_conflict,wprof_miss_capacity,wprof_miss_seen_alias);
            $display("PERF_DIAG|line_use|approx=1|one_word=%0d|two_words=%0d|three_words=%0d|four_words=%0d|hash_evictions=%0d",
                     wprof_lineuse_1word+wprof_tmp_1word,
                     wprof_lineuse_2word+wprof_tmp_2word,
                     wprof_lineuse_3word+wprof_tmp_3word,
                     wprof_lineuse_4word+wprof_tmp_4word,
                     wprof_lineuse_hash_evictions);
            for (wprof_i = 0; wprof_i < WPROF_MEMPC_SLOTS; wprof_i = wprof_i + 1) begin
                if (wprof_mempc_valid[wprof_i] && (wprof_mempc_access[wprof_i] != 0))
                    $display("PERF_DIAG|memory_pc|slot=%0d|pc=0x%08x|access=%0d|load=%0d|store=%0d|miss=%0d|same_line=%0d|next_line=%0d|constant_stride=%0d|last_stride=%0d|alias=%0d",
                             wprof_i,wprof_mempc_pc[wprof_i],wprof_mempc_access[wprof_i],
                             wprof_mempc_load[wprof_i],wprof_mempc_store[wprof_i],wprof_mempc_miss[wprof_i],
                             wprof_mempc_same_line[wprof_i],wprof_mempc_next_line[wprof_i],
                             wprof_mempc_const_stride[wprof_i],wprof_mempc_last_stride[wprof_i],
                             wprof_mempc_alias[wprof_i]);
            end
            $display("PERF_DIAG|nonblocking_potential|approx=1|refill_cycles=%0d|independent_alu=%0d|independent_branch=%0d|memory_candidate=%0d|same_line_candidate=%0d|load_use_dependency=%0d|ifq_buffered_cycles=%0d|ifq_entries_sum=%0d",
                     wprof_nb_refill_cycles,wprof_nb_independent_alu,wprof_nb_independent_branch,
                     wprof_nb_memory_candidate,wprof_nb_same_line_candidate,
                     wprof_nb_load_use_dependency,wprof_nb_ifq_buffered_cycles,wprof_nb_ifq_entries_sum);
            $display("PERF_DIAG|icache|enabled_4way=%0d|kb=%0d|ways=%0d|lookups=%0d|hits=%0d|misses=%0d|miss_rate_x10000=%0d|refill_lines=%0d|rdreq_cycles=%0d|refill_cycles=%0d|hit0=%0d|hit1=%0d|hit2=%0d|hit3=%0d|victim0=%0d|victim1=%0d|victim2=%0d|victim3=%0d",
                     ICACHE_FOUR_WAY, ICACHE_FOUR_WAY ? 16 : 8, ICACHE_FOUR_WAY ? 4 : 2,
                     u_icache.sim_lookup_cnt-wprof_ic_lookup_base,
                     u_icache.sim_hit_cnt-wprof_ic_hit_base,
                     u_icache.sim_miss_cnt-wprof_ic_miss_base,
                     diag_ratio_x10000(u_icache.sim_miss_cnt-wprof_ic_miss_base,
                                       u_icache.sim_lookup_cnt-wprof_ic_lookup_base),
                     u_icache.sim_refill_line_cnt-wprof_ic_refill_line_base,
                     u_icache.sim_rdreq_cycle_cnt-wprof_ic_rdreq_cycle_base,
                     u_icache.sim_refill_cycle_cnt-wprof_ic_refill_cycle_base,
                     u_icache.sim_way0_hit_cnt-wprof_ic_way0_hit_base,
                     u_icache.sim_way1_hit_cnt-wprof_ic_way1_hit_base,
                     u_icache.sim_way2_hit_cnt-wprof_ic_way2_hit_base,
                     u_icache.sim_way3_hit_cnt-wprof_ic_way3_hit_base,
                     u_icache.sim_way0_victim_cnt-wprof_ic_way0_victim_base,
                     u_icache.sim_way1_victim_cnt-wprof_ic_way1_victim_base,
                     u_icache.sim_way2_victim_cnt-wprof_ic_way2_victim_base,
                     u_icache.sim_way3_victim_cnt-wprof_ic_way3_victim_base);
`ifdef DISABLE_M170C_4WAY_DCACHE
            $display("PERF_DIAG|dcache_geometry|kb=8|ways=2|sets=256|line_bytes=16|m175d=0");
`else
`ifdef DISABLE_M175D_32KB_DCACHE
            $display("PERF_DIAG|dcache_geometry|kb=16|ways=4|sets=256|line_bytes=16|m175d=0");
`else
            $display("PERF_DIAG|dcache_geometry|kb=32|ways=4|sets=512|line_bytes=16|m175d=1");
`endif
`endif
            $display("PERF_DIAG|dcache|enabled_32kb=%0d|kb=%0d|ways=%0d|sets=%0d|lookups=%0d|hits=%0d|misses=%0d|miss_rate_x10000=%0d|refill_lines=%0d|rdreq_cycles=%0d|refill_cycles=%0d|hit0=%0d|hit1=%0d|hit2=%0d|hit3=%0d|victim0=%0d|victim1=%0d|victim2=%0d|victim3=%0d",
                     DCACHE_SETS_512,
                     DCACHE_FOUR_WAY ? (DCACHE_SETS_512 ? 32 : 16) : 8,
                     DCACHE_FOUR_WAY ? 4 : 2,
                     DCACHE_SETS_512 ? 512 : 256,
                     u_dcache.sim_lookup_cnt-wprof_dc_lookup_base,
                     u_dcache.sim_hit_cnt-wprof_dc_hit_base,
                     u_dcache.sim_miss_cnt-wprof_dc_miss_base,
                     diag_ratio_x10000(u_dcache.sim_miss_cnt-wprof_dc_miss_base,
                                       u_dcache.sim_lookup_cnt-wprof_dc_lookup_base),
                     u_dcache.sim_refill_line_cnt-wprof_dc_refill_line_base,
                     u_dcache.sim_rdreq_cycle_cnt-wprof_dc_rdreq_cycle_base,
                     u_dcache.sim_refill_cycle_cnt-wprof_dc_refill_cycle_base,
                     u_dcache.sim_way0_hit_cnt-wprof_dc_way0_hit_base,
                     u_dcache.sim_way1_hit_cnt-wprof_dc_way1_hit_base,
                     u_dcache.sim_way2_hit_cnt-wprof_dc_way2_hit_base,
                     u_dcache.sim_way3_hit_cnt-wprof_dc_way3_hit_base,
                     u_dcache.sim_way0_victim_cnt-wprof_dc_way0_victim_base,
                     u_dcache.sim_way1_victim_cnt-wprof_dc_way1_victim_base,
                     u_dcache.sim_way2_victim_cnt-wprof_dc_way2_victim_base,
                     u_dcache.sim_way3_victim_cnt-wprof_dc_way3_victim_base);
            $display("PERF_DIAG|hit_under_refill|enabled=%0d|accepted=%0d|same_line_hits=%0d|other_line_hits=%0d|second_misses=%0d|replay_setups=%0d|replay_hits=%0d|replay_misses=%0d|word_wait_cycles=%0d|store_block_cycles=%0d|useful=%0d|useful_x10000=%0d",
                     DCACHE_HIT_UNDER_REFILL,
                     u_dcache.hur_accept_cnt-wprof_hur_accept_base,
                     u_dcache.hur_same_line_hit_cnt-wprof_hur_same_line_base,
                     u_dcache.hur_other_line_hit_cnt-wprof_hur_other_line_base,
                     u_dcache.hur_second_miss_cnt-wprof_hur_second_miss_base,
                     u_dcache.hur_replay_setup_cnt-wprof_hur_replay_setup_base,
                     u_dcache.hur_replay_hit_cnt-wprof_hur_replay_hit_base,
                     u_dcache.hur_replay_miss_cnt-wprof_hur_replay_miss_base,
                     u_dcache.hur_word_wait_cycle_cnt-wprof_hur_word_wait_base,
                     u_dcache.hur_store_block_cycle_cnt-wprof_hur_store_block_base,
                     (u_dcache.hur_same_line_hit_cnt-wprof_hur_same_line_base)+
                     (u_dcache.hur_other_line_hit_cnt-wprof_hur_other_line_base),
                     diag_ratio_x10000(
                       (u_dcache.hur_same_line_hit_cnt-wprof_hur_same_line_base)+
                       (u_dcache.hur_other_line_hit_cnt-wprof_hur_other_line_base),
                       u_dcache.hur_accept_cnt-wprof_hur_accept_base));
            $display("PERF_DIAG_END|version=M17.5D-D12|segment=%0d", wprof_segment_index);
            wprof_segment_index <= wprof_segment_index + 32'd1;
        end else begin
            wprof_active <= 1'b1;
        end

        // Snapshot cumulative DCache HUR counters for the next segment.
        wprof_hur_accept_base <= u_dcache.hur_accept_cnt;
        wprof_hur_same_line_base <= u_dcache.hur_same_line_hit_cnt;
        wprof_hur_other_line_base <= u_dcache.hur_other_line_hit_cnt;
        wprof_hur_second_miss_base <= u_dcache.hur_second_miss_cnt;
        wprof_hur_replay_setup_base <= u_dcache.hur_replay_setup_cnt;
        wprof_hur_replay_hit_base <= u_dcache.hur_replay_hit_cnt;
        wprof_hur_replay_miss_base <= u_dcache.hur_replay_miss_cnt;
        wprof_hur_word_wait_base <= u_dcache.hur_word_wait_cycle_cnt;
        wprof_hur_store_block_base <= u_dcache.hur_store_block_cycle_cnt;
        wprof_ic_lookup_base <= u_icache.sim_lookup_cnt;
        wprof_ic_hit_base <= u_icache.sim_hit_cnt;
        wprof_ic_miss_base <= u_icache.sim_miss_cnt;
        wprof_ic_refill_line_base <= u_icache.sim_refill_line_cnt;
        wprof_ic_rdreq_cycle_base <= u_icache.sim_rdreq_cycle_cnt;
        wprof_ic_refill_cycle_base <= u_icache.sim_refill_cycle_cnt;
        wprof_ic_way0_hit_base <= u_icache.sim_way0_hit_cnt;
        wprof_ic_way1_hit_base <= u_icache.sim_way1_hit_cnt;
        wprof_ic_way2_hit_base <= u_icache.sim_way2_hit_cnt;
        wprof_ic_way3_hit_base <= u_icache.sim_way3_hit_cnt;
        wprof_ic_way0_victim_base <= u_icache.sim_way0_victim_cnt;
        wprof_ic_way1_victim_base <= u_icache.sim_way1_victim_cnt;
        wprof_ic_way2_victim_base <= u_icache.sim_way2_victim_cnt;
        wprof_ic_way3_victim_base <= u_icache.sim_way3_victim_cnt;
        wprof_dc_lookup_base <= u_dcache.sim_lookup_cnt;
        wprof_dc_hit_base <= u_dcache.sim_hit_cnt;
        wprof_dc_miss_base <= u_dcache.sim_miss_cnt;
        wprof_dc_refill_line_base <= u_dcache.sim_refill_line_cnt;
        wprof_dc_rdreq_cycle_base <= u_dcache.sim_rdreq_cycle_cnt;
        wprof_dc_refill_cycle_base <= u_dcache.sim_refill_cycle_cnt;
        wprof_dc_way0_hit_base <= u_dcache.sim_way0_hit_cnt;
        wprof_dc_way1_hit_base <= u_dcache.sim_way1_hit_cnt;
        wprof_dc_way2_hit_base <= u_dcache.sim_way2_hit_cnt;
        wprof_dc_way3_hit_base <= u_dcache.sim_way3_hit_cnt;
        wprof_dc_way0_victim_base <= u_dcache.sim_way0_victim_cnt;
        wprof_dc_way1_victim_base <= u_dcache.sim_way1_victim_cnt;
        wprof_dc_way2_victim_base <= u_dcache.sim_way2_victim_cnt;
        wprof_dc_way3_victim_base <= u_dcache.sim_way3_victim_cnt;

        // Clear only segment-local counters/tables.  The profiler remains armed
        // so the next adjacent timer-read interval is measured automatically.
        wprof_cycles <= 32'b0;
        wprof_issue_cycles <= 32'b0;
        wprof_issued_insts <= 32'b0;
        wprof_dual_cycles <= 32'b0;
        wprof_redirects <= 32'b0;
        wprof_pair_fail_raw <= 32'b0;
        wprof_pair_fail_waw <= 32'b0;
        wprof_pair_fail_structural <= 32'b0;
        wprof_pair_fail_unsupported <= 32'b0;
        wprof_pair_fail_older_not_ready <= 32'b0;
        wprof_pair_fail_boundary <= 32'b0;
        wprof_cpi_productive <= 32'b0;
        wprof_cpi_frontend_empty <= 32'b0;
        wprof_cpi_icache_wait <= 32'b0;
        wprof_cpi_branch_recovery <= 32'b0;
        wprof_cpi_decode_dependency <= 32'b0;
        wprof_cpi_load_use <= 32'b0;
        wprof_cpi_mul_wait <= 32'b0;
        wprof_cpi_div_wait <= 32'b0;
        wprof_cpi_dcache_lookup <= 32'b0;
        wprof_cpi_dcache_rdreq <= 32'b0;
        wprof_cpi_dcache_refill <= 32'b0;
        wprof_cpi_dcache_wb <= 32'b0;
        wprof_cpi_uncached <= 32'b0;
        wprof_cpi_backend_structural <= 32'b0;
        wprof_cpi_other <= 32'b0;
        wprof_branch_alias_events <= 32'b0;
        wprof_call_count <= 32'b0;
        wprof_return_count <= 32'b0;
        wprof_indirect_count <= 32'b0;
        wprof_return_target_miss <= 32'b0;
        wprof_indirect_target_miss <= 32'b0;
        wprof_miss_compulsory <= 32'b0;
        wprof_miss_conflict <= 32'b0;
        wprof_miss_capacity <= 32'b0;
        wprof_miss_seen_alias <= 32'b0;
        wprof_recent_ptr <= 10'b0;
        wprof_mempc_alias_events <= 32'b0;
        wprof_lineuse_1word <= 32'b0;
        wprof_lineuse_2word <= 32'b0;
        wprof_lineuse_3word <= 32'b0;
        wprof_lineuse_4word <= 32'b0;
        wprof_lineuse_hash_evictions <= 32'b0;
        wprof_nb_refill_cycles <= 32'b0;
        wprof_nb_independent_alu <= 32'b0;
        wprof_nb_independent_branch <= 32'b0;
        wprof_nb_memory_candidate <= 32'b0;
        wprof_nb_same_line_candidate <= 32'b0;
        wprof_nb_load_use_dependency <= 32'b0;
        wprof_nb_ifq_buffered_cycles <= 32'b0;
        wprof_nb_ifq_entries_sum <= 32'b0;
        for (wprof_i = 0; wprof_i < WPROF_MIX_CLASSES; wprof_i = wprof_i + 1)
            wprof_mix[wprof_i] <= 32'b0;
        for (wprof_i = 0; wprof_i < WPROF_PAIR_CLASSES*WPROF_PAIR_CLASSES; wprof_i = wprof_i + 1)
            wprof_pair_matrix[wprof_i] <= 32'b0;
        for (wprof_i = 0; wprof_i < WPROF_BRANCH_SLOTS; wprof_i = wprof_i + 1) begin
            wprof_br_valid[wprof_i] <= 1'b0;
            wprof_br_exec[wprof_i] <= 32'b0;
            wprof_br_taken[wprof_i] <= 32'b0;
            wprof_br_miss[wprof_i] <= 32'b0;
            wprof_br_dir_miss[wprof_i] <= 32'b0;
            wprof_br_tgt_miss[wprof_i] <= 32'b0;
            wprof_br_forward[wprof_i] <= 32'b0;
            wprof_br_backward[wprof_i] <= 32'b0;
            wprof_br_flips[wprof_i] <= 32'b0;
            wprof_br_alias[wprof_i] <= 32'b0;
            wprof_br_run_len[wprof_i] <= 16'b0;
            wprof_br_max_taken_run[wprof_i] <= 16'b0;
            wprof_br_max_nt_run[wprof_i] <= 16'b0;
        end
        for (wprof_i = 0; wprof_i < WPROF_MEMPC_SLOTS; wprof_i = wprof_i + 1) begin
            wprof_mempc_valid[wprof_i] <= 1'b0;
            wprof_mempc_access[wprof_i] <= 32'b0;
            wprof_mempc_load[wprof_i] <= 32'b0;
            wprof_mempc_store[wprof_i] <= 32'b0;
            wprof_mempc_miss[wprof_i] <= 32'b0;
            wprof_mempc_same_line[wprof_i] <= 32'b0;
            wprof_mempc_next_line[wprof_i] <= 32'b0;
            wprof_mempc_const_stride[wprof_i] <= 32'b0;
            wprof_mempc_alias[wprof_i] <= 32'b0;
            wprof_mempc_have_last[wprof_i] <= 1'b0;
        end
        for (wprof_i = 0; wprof_i < WPROF_SEEN_SLOTS; wprof_i = wprof_i + 1)
            wprof_seen_valid[wprof_i] <= 1'b0;
        for (wprof_i = 0; wprof_i < WPROF_RECENT_LINES; wprof_i = wprof_i + 1)
            wprof_recent_valid[wprof_i] <= 1'b0;
        for (wprof_i = 0; wprof_i < WPROF_LINEUSE_SLOTS; wprof_i = wprof_i + 1)
            wprof_lineuse_valid[wprof_i] <= 1'b0;
    end else if (wprof_active) begin
        wprof_cycles <= wprof_cycles + 32'd1;

        // Instruction mix and issue width.
        if (diag_main_issue) begin
            wprof_issue_cycles <= wprof_issue_cycles + 32'd1;
            wprof_issued_insts <= wprof_issued_insts + (diag_pair_success ? 32'd2 : 32'd1);
            if (diag_pair_success)
                wprof_dual_cycles <= wprof_dual_cycles + 32'd1;
            for (wprof_i = 0; wprof_i < WPROF_MIX_CLASSES; wprof_i = wprof_i + 1)
                wprof_mix[wprof_i] <= wprof_mix[wprof_i] +
                    ((wprof_main_mix_class == wprof_i) ? 32'd1 : 32'd0) +
                    ((diag_pair_success && (wprof_young_mix_class == wprof_i)) ? 32'd1 : 32'd0);
        end
        if (diag_redirect_fire)
            wprof_redirects <= wprof_redirects + 32'd1;

        // Pair opportunity matrix and mutually exclusive failure reason.
        if (diag_main_issue && diag_pair_has_next) begin
            wprof_matrix_idx = wprof_old_pair_class*WPROF_PAIR_CLASSES + wprof_young_pair_class;
            wprof_pair_matrix[wprof_matrix_idx] <= wprof_pair_matrix[wprof_matrix_idx] + 32'd1;
            if (!diag_pair_success) begin
                if (!diag_pair_adjacent || !diag_pair_same_page)
                    wprof_pair_fail_boundary <= wprof_pair_fail_boundary + 32'd1;
                else if (diag_dep_pair_raw)
                    wprof_pair_fail_raw <= wprof_pair_fail_raw + 32'd1;
                else if (diag_dep_pair_waw)
                    wprof_pair_fail_waw <= wprof_pair_fail_waw + 32'd1;
                else if (!u_mycpu_core.id_stage.es_allowin)
                    wprof_pair_fail_older_not_ready <= wprof_pair_fail_older_not_ready + 32'd1;
                else if (diag_normal_dependency || diag_swap_dependency)
                    wprof_pair_fail_structural <= wprof_pair_fail_structural + 32'd1;
                else
                    wprof_pair_fail_unsupported <= wprof_pair_fail_unsupported + 32'd1;
            end
        end

        // CPI stack: exactly one bucket is incremented per active cycle.
        if (diag_mem_waiting && diag_mem_wait_uncache)
            wprof_cpi_uncached <= wprof_cpi_uncached + 32'd1;
        else if (diag_mem_waiting && diag_dcache_refill_state)
            wprof_cpi_dcache_refill <= wprof_cpi_dcache_refill + 32'd1;
        else if (diag_mem_waiting && diag_dcache_rdreq_state)
            wprof_cpi_dcache_rdreq <= wprof_cpi_dcache_rdreq + 32'd1;
        else if (diag_mem_waiting && diag_dcache_wb_state)
            wprof_cpi_dcache_wb <= wprof_cpi_dcache_wb + 32'd1;
        else if (diag_mem_waiting && diag_dcache_lookup)
            wprof_cpi_dcache_lookup <= wprof_cpi_dcache_lookup + 32'd1;
        else if (diag_redirect_fire)
            wprof_cpi_branch_recovery <= wprof_cpi_branch_recovery + 32'd1;
        else if (u_mycpu_core.exe_stage.es_valid && u_mycpu_core.exe_stage.inst_div_op &&
                 !u_mycpu_core.exe_stage.div_done)
            wprof_cpi_div_wait <= wprof_cpi_div_wait + 32'd1;
        else if (u_mycpu_core.exe_stage.es_valid && u_mycpu_core.exe_stage.inst_mul_op &&
                 !u_mycpu_core.exe_stage.mul_product_valid)
            wprof_cpi_mul_wait <= wprof_cpi_mul_wait + 32'd1;
        else if (u_mycpu_core.perf_id_load_stall || u_mycpu_core.perf_id_ms_load_stall)
            wprof_cpi_load_use <= wprof_cpi_load_use + 32'd1;
        else if (u_mycpu_core.perf_id_branch_src_stall || u_mycpu_core.perf_id_es_raw_stall ||
                 u_mycpu_core.perf_id_mem_addr_es_stall || u_mycpu_core.perf_id_tlb_precheck_stall)
            wprof_cpi_decode_dependency <= wprof_cpi_decode_dependency + 32'd1;
        else if (inst_sram_req && (!inst_sram_addr_ok || !inst_sram_data_ok))
            wprof_cpi_icache_wait <= wprof_cpi_icache_wait + 32'd1;
        else if (!u_mycpu_core.id_stage.ds_valid && (diag_ifq_count == 0))
            wprof_cpi_frontend_empty <= wprof_cpi_frontend_empty + 32'd1;
        else if (diag_main_issue)
            wprof_cpi_productive <= wprof_cpi_productive + 32'd1;
        else if (u_mycpu_core.id_stage.ds_valid && !u_mycpu_core.id_stage.es_allowin)
            wprof_cpi_backend_structural <= wprof_cpi_backend_structural + 32'd1;
        else
            wprof_cpi_other <= wprof_cpi_other + 32'd1;

        // Branch table.
        if (diag_branch_update) begin
            if (!wprof_br_valid[wprof_branch_idx] ||
                (wprof_br_pc[wprof_branch_idx] != diag_branch_pc)) begin
                if (wprof_br_valid[wprof_branch_idx]) begin
                    wprof_branch_alias_events <= wprof_branch_alias_events + 32'd1;
                    wprof_br_alias[wprof_branch_idx] <= wprof_br_alias[wprof_branch_idx] + 32'd1;
                end
                wprof_br_valid[wprof_branch_idx] <= 1'b1;
                wprof_br_pc[wprof_branch_idx] <= diag_branch_pc;
                wprof_br_type[wprof_branch_idx] <= wprof_branch_type;
                wprof_br_exec[wprof_branch_idx] <= 32'd1;
                wprof_br_taken[wprof_branch_idx] <= diag_branch_taken ? 32'd1 : 32'd0;
                wprof_br_miss[wprof_branch_idx] <= diag_branch_miss ? 32'd1 : 32'd0;
                wprof_br_dir_miss[wprof_branch_idx] <= wprof_branch_direction_miss ? 32'd1 : 32'd0;
                wprof_br_tgt_miss[wprof_branch_idx] <= wprof_branch_target_miss ? 32'd1 : 32'd0;
                wprof_br_forward[wprof_branch_idx] <= (!diag_branch_backward) ? 32'd1 : 32'd0;
                wprof_br_backward[wprof_branch_idx] <= diag_branch_backward ? 32'd1 : 32'd0;
                wprof_br_flips[wprof_branch_idx] <= 32'b0;
                wprof_br_bht_index[wprof_branch_idx] <= wprof_actual_bht_idx;
                wprof_br_last_taken[wprof_branch_idx] <= diag_branch_taken;
                wprof_br_run_len[wprof_branch_idx] <= 16'd1;
                wprof_br_max_taken_run[wprof_branch_idx] <= diag_branch_taken ? 16'd1 : 16'd0;
                wprof_br_max_nt_run[wprof_branch_idx] <= diag_branch_taken ? 16'd0 : 16'd1;
            end else begin
                wprof_br_exec[wprof_branch_idx] <= wprof_br_exec[wprof_branch_idx] + 32'd1;
                if (diag_branch_taken)
                    wprof_br_taken[wprof_branch_idx] <= wprof_br_taken[wprof_branch_idx] + 32'd1;
                if (diag_branch_miss)
                    wprof_br_miss[wprof_branch_idx] <= wprof_br_miss[wprof_branch_idx] + 32'd1;
                if (wprof_branch_direction_miss)
                    wprof_br_dir_miss[wprof_branch_idx] <= wprof_br_dir_miss[wprof_branch_idx] + 32'd1;
                if (wprof_branch_target_miss)
                    wprof_br_tgt_miss[wprof_branch_idx] <= wprof_br_tgt_miss[wprof_branch_idx] + 32'd1;
                if (diag_branch_backward)
                    wprof_br_backward[wprof_branch_idx] <= wprof_br_backward[wprof_branch_idx] + 32'd1;
                else
                    wprof_br_forward[wprof_branch_idx] <= wprof_br_forward[wprof_branch_idx] + 32'd1;
                if (wprof_br_last_taken[wprof_branch_idx] != diag_branch_taken) begin
                    wprof_br_flips[wprof_branch_idx] <= wprof_br_flips[wprof_branch_idx] + 32'd1;
                    wprof_br_run_len[wprof_branch_idx] <= 16'd1;
                end else begin
                    wprof_br_run_len[wprof_branch_idx] <= wprof_br_run_len[wprof_branch_idx] + 16'd1;
                end
                if (diag_branch_taken &&
                    ((wprof_br_last_taken[wprof_branch_idx] ? wprof_br_run_len[wprof_branch_idx]+16'd1 : 16'd1) >
                     wprof_br_max_taken_run[wprof_branch_idx]))
                    wprof_br_max_taken_run[wprof_branch_idx] <=
                        wprof_br_last_taken[wprof_branch_idx] ? wprof_br_run_len[wprof_branch_idx]+16'd1 : 16'd1;
                if (!diag_branch_taken &&
                    ((!wprof_br_last_taken[wprof_branch_idx] ? wprof_br_run_len[wprof_branch_idx]+16'd1 : 16'd1) >
                     wprof_br_max_nt_run[wprof_branch_idx]))
                    wprof_br_max_nt_run[wprof_branch_idx] <=
                        !wprof_br_last_taken[wprof_branch_idx] ? wprof_br_run_len[wprof_branch_idx]+16'd1 : 16'd1;
                wprof_br_last_taken[wprof_branch_idx] <= diag_branch_taken;
            end
            if (u_mycpu_core.id_stage.bp_update_is_call)
                wprof_call_count <= wprof_call_count + 32'd1;
            if (u_mycpu_core.id_stage.bp_update_is_return) begin
                wprof_return_count <= wprof_return_count + 32'd1;
                if (wprof_branch_target_miss)
                    wprof_return_target_miss <= wprof_return_target_miss + 32'd1;
            end
            if (u_mycpu_core.id_stage.bp_update_is_indirect) begin
                wprof_indirect_count <= wprof_indirect_count + 32'd1;
                if (wprof_branch_target_miss)
                    wprof_indirect_target_miss <= wprof_indirect_target_miss + 32'd1;
            end
        end

        // Per-PC address stride and line utilization are sampled once per DCache lookup.
        if (diag_dcache_lookup && !u_dcache.cacop_valid) begin
            if (!wprof_mempc_valid[wprof_mempc_idx] ||
                (wprof_mempc_pc[wprof_mempc_idx] != diag_mem_wait_pc)) begin
                if (wprof_mempc_valid[wprof_mempc_idx]) begin
                    wprof_mempc_alias_events <= wprof_mempc_alias_events + 32'd1;
                    wprof_mempc_alias[wprof_mempc_idx] <= wprof_mempc_alias[wprof_mempc_idx] + 32'd1;
                end
                wprof_mempc_valid[wprof_mempc_idx] <= 1'b1;
                wprof_mempc_pc[wprof_mempc_idx] <= diag_mem_wait_pc;
                wprof_mempc_access[wprof_mempc_idx] <= 32'd1;
                wprof_mempc_load[wprof_mempc_idx] <= diag_dcache_req_is_store ? 32'd0 : 32'd1;
                wprof_mempc_store[wprof_mempc_idx] <= diag_dcache_req_is_store ? 32'd1 : 32'd0;
                wprof_mempc_miss[wprof_mempc_idx] <= diag_dcache_lookup_miss ? 32'd1 : 32'd0;
                wprof_mempc_same_line[wprof_mempc_idx] <= 32'b0;
                wprof_mempc_next_line[wprof_mempc_idx] <= 32'b0;
                wprof_mempc_const_stride[wprof_mempc_idx] <= 32'b0;
                wprof_mempc_last_addr[wprof_mempc_idx] <= wprof_dcache_addr;
                wprof_mempc_last_stride[wprof_mempc_idx] <= 32'b0;
                wprof_mempc_have_last[wprof_mempc_idx] <= 1'b1;
            end else begin
                wprof_mempc_access[wprof_mempc_idx] <= wprof_mempc_access[wprof_mempc_idx] + 32'd1;
                if (diag_dcache_req_is_store)
                    wprof_mempc_store[wprof_mempc_idx] <= wprof_mempc_store[wprof_mempc_idx] + 32'd1;
                else
                    wprof_mempc_load[wprof_mempc_idx] <= wprof_mempc_load[wprof_mempc_idx] + 32'd1;
                if (diag_dcache_lookup_miss)
                    wprof_mempc_miss[wprof_mempc_idx] <= wprof_mempc_miss[wprof_mempc_idx] + 32'd1;
                if (wprof_mempc_have_last[wprof_mempc_idx]) begin
                    wprof_stride_now = wprof_dcache_addr - wprof_mempc_last_addr[wprof_mempc_idx];
                    if (wprof_dcache_addr[31:4] == wprof_mempc_last_addr[wprof_mempc_idx][31:4])
                        wprof_mempc_same_line[wprof_mempc_idx] <= wprof_mempc_same_line[wprof_mempc_idx] + 32'd1;
                    if (wprof_stride_now == 32'd16)
                        wprof_mempc_next_line[wprof_mempc_idx] <= wprof_mempc_next_line[wprof_mempc_idx] + 32'd1;
                    if (wprof_stride_now == wprof_mempc_last_stride[wprof_mempc_idx])
                        wprof_mempc_const_stride[wprof_mempc_idx] <= wprof_mempc_const_stride[wprof_mempc_idx] + 32'd1;
                    wprof_mempc_last_stride[wprof_mempc_idx] <= wprof_stride_now;
                end
                wprof_mempc_last_addr[wprof_mempc_idx] <= wprof_dcache_addr;
            end

            if (!wprof_lineuse_valid[wprof_lineuse_index(wprof_dcache_line)] ||
                (wprof_lineuse_line[wprof_lineuse_index(wprof_dcache_line)] == wprof_dcache_line)) begin
                wprof_lineuse_valid[wprof_lineuse_index(wprof_dcache_line)] <= 1'b1;
                wprof_lineuse_line[wprof_lineuse_index(wprof_dcache_line)] <= wprof_dcache_line;
                wprof_lineuse_mask[wprof_lineuse_index(wprof_dcache_line)] <=
                    wprof_lineuse_mask[wprof_lineuse_index(wprof_dcache_line)] | wprof_dcache_word_bit;
            end else begin
                wprof_words_used = wprof_popcount4(wprof_lineuse_mask[wprof_lineuse_index(wprof_dcache_line)]);
                if (wprof_words_used == 3'd1) wprof_lineuse_1word <= wprof_lineuse_1word + 32'd1;
                else if (wprof_words_used == 3'd2) wprof_lineuse_2word <= wprof_lineuse_2word + 32'd1;
                else if (wprof_words_used == 3'd3) wprof_lineuse_3word <= wprof_lineuse_3word + 32'd1;
                else if (wprof_words_used == 3'd4) wprof_lineuse_4word <= wprof_lineuse_4word + 32'd1;
                wprof_lineuse_hash_evictions <= wprof_lineuse_hash_evictions + 32'd1;
                wprof_lineuse_line[wprof_lineuse_index(wprof_dcache_line)] <= wprof_dcache_line;
                wprof_lineuse_mask[wprof_lineuse_index(wprof_dcache_line)] <= wprof_dcache_word_bit;
            end
        end

        // Approximate compulsory/conflict/capacity taxonomy on each actual miss.
        if (diag_dcache_lookup_miss && !u_dcache.cacop_valid) begin
            wprof_recent_found = 0;
            for (wprof_j = 0; wprof_j < WPROF_RECENT_LINES; wprof_j = wprof_j + 1)
                if (wprof_recent_valid[wprof_j] && (wprof_recent_line[wprof_j] == wprof_dcache_line))
                    wprof_recent_found = 1;
            if (!wprof_seen_valid[wprof_seen_index(wprof_dcache_line)]) begin
                wprof_miss_compulsory <= wprof_miss_compulsory + 32'd1;
                wprof_seen_valid[wprof_seen_index(wprof_dcache_line)] <= 1'b1;
                wprof_seen_line[wprof_seen_index(wprof_dcache_line)] <= wprof_dcache_line;
            end else if (wprof_seen_line[wprof_seen_index(wprof_dcache_line)] != wprof_dcache_line) begin
                wprof_miss_seen_alias <= wprof_miss_seen_alias + 32'd1;
                if (wprof_recent_found != 0)
                    wprof_miss_conflict <= wprof_miss_conflict + 32'd1;
                else
                    wprof_miss_capacity <= wprof_miss_capacity + 32'd1;
                wprof_seen_line[wprof_seen_index(wprof_dcache_line)] <= wprof_dcache_line;
            end else if (wprof_recent_found != 0) begin
                wprof_miss_conflict <= wprof_miss_conflict + 32'd1;
            end else begin
                wprof_miss_capacity <= wprof_miss_capacity + 32'd1;
            end
            wprof_recent_valid[wprof_recent_ptr] <= 1'b1;
            wprof_recent_line[wprof_recent_ptr] <= wprof_dcache_line;
            wprof_recent_ptr <= wprof_recent_ptr + 10'd1;
        end

        // Lightweight estimate of useful work hidden by a future background refill.
        if (diag_dcache_refill_state) begin
            wprof_nb_refill_cycles <= wprof_nb_refill_cycles + 32'd1;
            if (diag_ifq_count != 0) begin
                wprof_nb_ifq_buffered_cycles <= wprof_nb_ifq_buffered_cycles + 32'd1;
                wprof_nb_ifq_entries_sum <= wprof_nb_ifq_entries_sum + {29'b0,diag_ifq_count};
            end
            if (u_mycpu_core.id_stage.ds_valid) begin
                if (u_mycpu_core.perf_id_ms_load_stall)
                    wprof_nb_load_use_dependency <= wprof_nb_load_use_dependency + 32'd1;
                else if (u_mycpu_core.id_stage.current_simple_alu)
                    wprof_nb_independent_alu <= wprof_nb_independent_alu + 32'd1;
                else if (u_mycpu_core.id_stage.ctrl_transfer_inst)
                    wprof_nb_independent_branch <= wprof_nb_independent_branch + 32'd1;
                else if (u_mycpu_core.id_stage.ds_mem_access) begin
                    wprof_nb_memory_candidate <= wprof_nb_memory_candidate + 32'd1;
                    if (u_mycpu_core.id_stage.mem_addr_id[31:4] == wprof_dcache_addr[31:4])
                        wprof_nb_same_line_candidate <= wprof_nb_same_line_candidate + 32'd1;
                end
            end
        end
    end
end
