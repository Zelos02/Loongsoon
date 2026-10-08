`include "mycpu.h"

// 100 MHz-oriented timing/IPC optimization build:
// 1) TLB precheck payload/result registers use unconditional data capture,
//    removing the large decode/hazard cone from hundreds of register CEs.
// 2) Branch predictor training is pipelined by one cycle, removing the
//    current-ID decode/branch cone from BTB/BHT/GHR write controls.

//------------------------------------------------------------------------------
// exp16 AXI top wrapper
// Keep the exp14 SRAM-like CPU core unchanged, and put a 2x1 SRAM-like to AXI
// bridge outside it.  The CPU exposes only one AXI master interface to soc_axi.
//------------------------------------------------------------------------------
module mycpu_top(
    input  wire        aclk,
    input  wire        aresetn,

    // read address channel
    output wire [ 3:0] arid,
    output wire [31:0] araddr,
    output wire [ 7:0] arlen,
    output wire [ 2:0] arsize,
    output wire [ 1:0] arburst,
    output wire [ 1:0] arlock,
    output wire [ 3:0] arcache,
    output wire [ 2:0] arprot,
    output wire        arvalid,
    input  wire        arready,

    // read data channel
    input  wire [ 3:0] rid,
    input  wire [31:0] rdata,
    input  wire [ 1:0] rresp,
    input  wire        rlast,
    input  wire        rvalid,
    output wire        rready,

    // write address channel
    output wire [ 3:0] awid,
    output wire [31:0] awaddr,
    output wire [ 7:0] awlen,
    output wire [ 2:0] awsize,
    output wire [ 1:0] awburst,
    output wire [ 1:0] awlock,
    output wire [ 3:0] awcache,
    output wire [ 2:0] awprot,
    output wire        awvalid,
    input  wire        awready,

    // write data channel
    output wire [ 3:0] wid,
    output wire [31:0] wdata,
    output wire [ 3:0] wstrb,
    output wire        wlast,
    output wire        wvalid,
    input  wire        wready,

    // write response channel
    input  wire [ 3:0] bid,
    input  wire [ 1:0] bresp,
    input  wire        bvalid,
    output wire        bready,

    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata
);

wire clk;
wire resetn;
assign clk    = aclk;
assign resetn = aresetn;

// Internal SRAM-like instruction interface.
wire        inst_sram_req;
wire        inst_sram_wr;
wire [ 1:0] inst_sram_size;
wire [ 3:0] inst_sram_wstrb;
wire [31:0] inst_sram_addr;
wire [31:0] inst_sram_wdata;
wire        inst_sram_dual;
wire        inst_sram_addr_ok;
wire        inst_sram_data_ok;
wire [31:0] inst_sram_rdata;
wire [31:0] inst_sram_rdata2;

// Internal SRAM-like data interface.
wire        data_sram_req;
wire        data_sram_wr;
wire [ 1:0] data_sram_size;
wire [ 3:0] data_sram_wstrb;
wire [31:0] data_sram_addr;
wire [31:0] data_sram_addr_fast;
wire        data_sram_addr_late;
wire [31:0] data_sram_wdata;
wire        data_sram_uncached;
wire        data_sram_addr_ok;
wire        data_sram_data_ok;
wire [31:0] data_sram_rdata;

// CACOP sideband from CPU core to ICache/DCache.
wire        cacop_req;
wire [ 4:0] cacop_code;
wire [31:0] cacop_addr;
wire [31:0] cacop_paddr;
wire        cacop_addr_ok;
wire        cacop_data_ok;
wire        icache_cacop_addr_ok;
wire        icache_cacop_data_ok;
wire        dcache_cacop_addr_ok;
wire        dcache_cacop_data_ok;
wire        cacop_to_icache;
wire        cacop_to_dcache;
wire [31:0] cacop_icache_paddr;
wire [31:0] cacop_dcache_paddr;
wire [ 4:0] icache_cacop_code_mux;
wire [31:0] icache_cacop_addr_mux;
wire [ 4:0] dcache_cacop_code_mux;
wire [31:0] dcache_cacop_addr_mux;

//------------------------------------------------------------------------------
// exp21 ICache integration
// CPU IF still sends a SRAM-like instruction request.  The ICache converts it to
// cache-line read requests, while data SRAM-like accesses still go directly to
// the AXI bridge.  The tag uses the same physical/AXI address mapping policy as
// exp19, so the already-passed MMU/DMW behavior is preserved.
//------------------------------------------------------------------------------
wire        icache_rd_req;
wire [ 2:0] icache_rd_type;
wire [31:0] icache_rd_addr;
wire        icache_rd_rdy;
wire        icache_ret_valid;
wire        icache_ret_last;
wire [31:0] icache_ret_data;

wire        icache_wr_req;
wire [ 2:0] icache_wr_type;
wire [31:0] icache_wr_addr;
wire [ 3:0] icache_wr_wstrb;
wire [127:0] icache_wr_data;
wire        icache_wr_rdy;

//------------------------------------------------------------------------------
// exp22 DCache integration
//
// The route of a load/store is decided by the LoongArch memory access type
// (MAT) produced by address translation, not by a physical-address decoder:
//
//   VA -> direct / DMW / TLB -> {PA, MAT} -> DCache or uncached AXI path.
//
// MAT=CC uses DCache; MAT=SUC/WUC bypasses DCache.  This keeps MMIO policy in
// CRMD/DMW/TLB software configuration, so newly mapped peripherals do not
// require edits in this top-level cache wrapper.
//------------------------------------------------------------------------------
wire        dcache_rd_req;
wire [ 2:0] dcache_rd_type;
wire [31:0] dcache_rd_addr;
wire        dcache_rd_rdy;
wire        dcache_ret_valid;
wire        dcache_ret_last;
wire [31:0] dcache_ret_data;

wire        dcache_wr_req;
wire [ 2:0] dcache_wr_type;
wire [31:0] dcache_wr_addr;
wire [ 3:0] dcache_wr_wstrb;
wire [127:0] dcache_wr_data;
wire        dcache_wr_rdy;

wire        dcache_cpu_addr_ok;
wire        dcache_cpu_data_ok;
wire [31:0] dcache_cpu_rdata;
wire        uncache_data_addr_ok;
wire        uncache_data_data_ok;
wire [31:0] uncache_data_rdata;

wire        data_uncache_req;
wire        data_cache_req;
wire        data_uncache_region;
wire        data_cache_req_accept;
wire        dcache_live_load;
wire        dcache_buffer_this_req;
wire        dcache_req_to_cache_valid;
wire        dcache_req_op_mux;
wire [31:0] dcache_req_addr_mux;
wire [ 3:0] dcache_req_wstrb_mux;
wire [31:0] dcache_req_wdata_mux;
reg         dcache_req_buf_valid;
reg         dcache_req_op_r;
reg  [31:0] dcache_req_addr_r;
reg  [ 3:0] dcache_req_wstrb_r;
reg  [31:0] dcache_req_wdata_r;
reg         dcache_early_store_ack_r;
reg  [ 1:0] dcache_store_resp_count;

// data_sram_uncached is generated in ID from the translated MAT and is
// carried with the request through EXE.  data_sram_addr is already physical
// here, but its value must not be used to infer cacheability.
assign data_uncache_region = data_sram_uncached;
assign data_uncache_req = data_sram_req && data_uncache_region;
assign data_cache_req   = data_sram_req && !data_uncache_region;
assign data_cache_req_accept = data_cache_req && !dcache_req_buf_valid;

// M19.10 selective cache-request timing boundary.
//
// A cached load whose base operand was already present in the EXE payload keeps
// the original zero-bubble fall-through.  Its dedicated fast address never
// contains a MEM/WB-forwarding mux.  Only a direct/DMW load with a late base
// operand, plus every store, enters the one-entry request register.  This cuts
// the forwarded-operand -> DCache BRAM path without imposing a cycle on the
// common independent-load case.
assign dcache_live_load = data_cache_req_accept && !data_sram_wr &&
                          !data_sram_addr_late;
assign dcache_buffer_this_req = data_sram_wr || data_sram_addr_late ||
                                !dcache_cpu_addr_ok;
assign dcache_req_to_cache_valid = dcache_req_buf_valid | dcache_live_load;
assign dcache_req_op_mux = dcache_req_buf_valid ? dcache_req_op_r :
                                                   data_sram_wr;
assign dcache_req_addr_mux = dcache_req_buf_valid ? dcache_req_addr_r :
                                                     data_sram_addr_fast;
assign dcache_req_wstrb_mux = dcache_req_buf_valid ? dcache_req_wstrb_r :
                                                      data_sram_wstrb;
assign dcache_req_wdata_mux = dcache_req_buf_valid ? dcache_req_wdata_r :
                                                      data_sram_wdata;

assign data_sram_addr_ok = data_uncache_req ? uncache_data_addr_ok :
                           data_cache_req   ? data_cache_req_accept : 1'b0;

`ifdef DISABLE_M187_EARLY_STORE_ACK
wire dcache_early_store_accept = 1'b0;
`else
wire dcache_early_store_accept = data_cache_req_accept && data_sram_wr;
`endif
wire dcache_drop_store_response =
     (dcache_store_resp_count != 2'b0);

always @(posedge clk) begin
    if (!resetn) begin
        dcache_req_buf_valid <= 1'b0;
        dcache_req_op_r      <= 1'b0;
        dcache_req_addr_r    <= 32'b0;
        dcache_req_wstrb_r   <= 4'b0;
        dcache_req_wdata_r   <= 32'b0;
        dcache_early_store_ack_r <= 1'b0;
        dcache_store_resp_count  <= 2'b0;
    end
    else begin
        // Delay the early acknowledgement by one cycle so the store already
        // occupies MEM when the pulse is observed.  This also keeps a load
        // response and a same-cycle following store request distinguishable.
        dcache_early_store_ack_r <= dcache_early_store_accept;

        // The cache and its one-entry skid register can hold at most two
        // early-retired stores.  Responses remain strictly ordered, so the
        // first matching number of DCache completion pulses can be discarded.
        case ({dcache_early_store_accept,
               dcache_cpu_data_ok && dcache_drop_store_response})
            2'b10: begin
                if (dcache_store_resp_count != 2'b11)
                    dcache_store_resp_count <= dcache_store_resp_count + 2'b01;
            end
            2'b01:
                dcache_store_resp_count <= dcache_store_resp_count - 2'b01;
            default:
                dcache_store_resp_count <= dcache_store_resp_count;
        endcase

        if (dcache_req_buf_valid && dcache_cpu_addr_ok) begin
            dcache_req_buf_valid <= 1'b0;
        end

        if (data_cache_req_accept) begin
            // A fast live load disappears into the cache immediately when
            // addr_ok is high.  Late-address loads and stores always use the
            // registered path; an unaccepted fast load is retained as a skid.
            if (dcache_buffer_this_req) begin
                dcache_req_buf_valid <= 1'b1;
                dcache_req_op_r      <= data_sram_wr;
                dcache_req_addr_r    <= data_sram_addr;
                dcache_req_wstrb_r   <= data_sram_wstrb;
                dcache_req_wdata_r   <= data_sram_wdata;
            end
            else begin
                dcache_req_buf_valid <= 1'b0;
            end
        end
    end
end

// exp22 v3 DRC fix:
// Do not feed the current EXE request back into the MEM-stage data_ok mux.
// The v2 expression used
//     data_resp_uncache | (data_sram_req && data_uncache_region)
// to select uncache_data_ok.  Because data_sram_data_ok affects ms_allowin,
// and ms_allowin gates data_sram_req/data_sram_addr in EXE, Vivado correctly
// reported a combinational loop.  Both DCache and uncached completion pulses
// are mutually exclusive in this single-outstanding bridge, so OR the two
// registered/completion-side data_ok pulses and select rdata by the active pulse.
assign data_sram_data_ok = uncache_data_data_ok |
                           dcache_early_store_ack_r |
                           (dcache_cpu_data_ok &&
                            !dcache_drop_store_response);
assign data_sram_rdata   = uncache_data_data_ok ? uncache_data_rdata : dcache_cpu_rdata;

// A younger cache-maintenance operation must not pass an early-retired store.
// Waiting for both the response counter and skid entry preserves program order
// for self-modifying code and DCache clean/invalidate operations.
wire dcache_store_order_clear =
     (dcache_store_resp_count == 2'b0) && !dcache_req_buf_valid &&
     !dcache_early_store_ack_r;
wire cacop_req_ordered = cacop_req && dcache_store_order_clear;
wire cacop_req_icache = cacop_req_ordered && (cacop_code[2:0] == 3'd0);
wire cacop_req_dcache = cacop_req_ordered && (cacop_code[2:0] == 3'd1);
wire cacop_to_other = cacop_req_ordered &&
                      !cacop_req_icache && !cacop_req_dcache;

// ICache CACOP is used by the self-modifying-code test.  The program stores
// new instructions through DCache and then invalidates ICache.  Because this
// DCache is write-back, clean the matching DCache line first so the following
// ICache refill sees the modified instruction in memory.
localparam ICACOP_IDLE        = 3'd0;
localparam ICACOP_DCLEAN_ADDR = 3'd1;
localparam ICACOP_DCLEAN_WAIT = 3'd2;
localparam ICACOP_I_ADDR      = 3'd3;
localparam ICACOP_I_WAIT      = 3'd4;

reg [2:0]  icacop_state;
reg [4:0]  icacop_code_r;
reg [31:0] icacop_addr_r;
reg [31:0] icacop_paddr_r;

// Direct DCache CACOP requests are buffered before driving cache RAM address
// ports.  The routed baseline had a 10.084 ns path from WB forwarding through
// EXE/CACOP address generation into DCache BRAM address pins.  CACOP is rare,
// so accepting it into a one-entry register costs one request cycle but removes
// that path from the normal CPU Fmax limit.
reg        dcacop_req_buf_valid;
reg [4:0]  dcacop_code_r;
reg [31:0] dcacop_addr_r;
reg [31:0] dcacop_paddr_r;

wire icacop_cpu_accept = (icacop_state == ICACOP_IDLE) && cacop_req_icache;
wire dcacop_cpu_accept = (icacop_state == ICACOP_IDLE) &&
                          cacop_req_dcache && !dcacop_req_buf_valid;
wire icacop_dclean_valid = (icacop_state == ICACOP_DCLEAN_ADDR);
wire icacop_icache_valid = (icacop_state == ICACOP_I_ADDR);
wire icacop_preclean_hit = (icacop_code_r[4:3] == 2'b10);
wire [1:0] icacop_preclean_mode = icacop_preclean_hit ? 2'b10 : 2'b01;

always @(posedge clk) begin
    if (!resetn) begin
        icacop_state  <= ICACOP_IDLE;
        icacop_code_r <= 5'b0;
        icacop_addr_r <= 32'b0;
        icacop_paddr_r <= 32'b0;
    end
    else begin
        case (icacop_state)
            ICACOP_IDLE: begin
                if (icacop_cpu_accept) begin
                    icacop_code_r <= cacop_code;
                    icacop_addr_r <= cacop_addr;
                    icacop_paddr_r <= cacop_paddr;
                    icacop_state  <= ICACOP_DCLEAN_ADDR;
                end
            end

            ICACOP_DCLEAN_ADDR: begin
                if (dcache_cacop_addr_ok) begin
                    icacop_state <= ICACOP_DCLEAN_WAIT;
                end
            end

            ICACOP_DCLEAN_WAIT: begin
                if (dcache_cacop_data_ok) begin
                    icacop_state <= ICACOP_I_ADDR;
                end
            end

            ICACOP_I_ADDR: begin
                if (icache_cacop_addr_ok) begin
                    icacop_state <= ICACOP_I_WAIT;
                end
            end

            ICACOP_I_WAIT: begin
                if (icache_cacop_data_ok) begin
                    icacop_state <= ICACOP_IDLE;
                end
            end

            default: begin
                icacop_state <= ICACOP_IDLE;
            end
        endcase
    end
end

always @(posedge clk) begin
    if (!resetn) begin
        dcacop_req_buf_valid <= 1'b0;
        dcacop_code_r        <= 5'b0;
        dcacop_addr_r        <= 32'b0;
        dcacop_paddr_r       <= 32'b0;
    end
    else begin
        if (dcacop_req_buf_valid && dcache_cacop_addr_ok)
            dcacop_req_buf_valid <= 1'b0;

        if (dcacop_cpu_accept) begin
            dcacop_req_buf_valid <= 1'b1;
            dcacop_code_r        <= cacop_code;
            dcacop_addr_r        <= cacop_addr;
            dcacop_paddr_r       <= cacop_paddr;
        end
    end
end

assign cacop_to_icache = icacop_icache_valid;
assign cacop_to_dcache = dcacop_req_buf_valid || icacop_dclean_valid;

assign icache_cacop_code_mux = icacop_code_r;
assign icache_cacop_addr_mux = icacop_addr_r;
// ICache StoreTag/Index CACOP carries only index+way, not a full tag.  Pre-clean
// DCache with the matching index operation so a dirty self-modified line is
// written back before the ICache line is invalidated/refilled.  Hit-mode CACOP
// keeps using the translated physical tag latched with the original request.
assign dcache_cacop_code_mux = icacop_dclean_valid ? {icacop_preclean_mode, 3'd1} : dcacop_code_r;
assign dcache_cacop_addr_mux = icacop_dclean_valid ? icacop_addr_r : dcacop_addr_r;

// Cache objects other than L1I/L1D are treated as no-ops in this lab CPU.
//
// Important exp23 fix:
//   cacop_req is generated in EXE and deasserts once addr_ok lets the CACOP
//   instruction enter MEM.  The cache returns cacop_data_ok one or more cycles
//   later.  Therefore data_ok must NOT be selected by the current cacop_req;
//   otherwise the MEM stage never sees the completion pulse and the CPU stalls
//   forever at the first CACOP.  The two cache-side completion pulses are
//   already mutually exclusive in this single-issue pipeline, so OR them here.
reg cacop_noop_data_ok_r;
always @(posedge clk) begin
    if (!resetn) begin
        cacop_noop_data_ok_r <= 1'b0;
    end
    else begin
        cacop_noop_data_ok_r <= cacop_to_other;
    end
end

assign cacop_addr_ok = !dcache_store_order_clear ? 1'b0 :
                       cacop_req_icache ? icacop_cpu_accept :
                       cacop_req_dcache ? dcacop_cpu_accept :
                       cacop_req_ordered;
assign cacop_data_ok = ((icacop_state == ICACOP_I_WAIT) && icache_cacop_data_ok) |
                       ((icacop_state == ICACOP_IDLE) && dcache_cacop_data_ok) |
                       cacop_noop_data_ok_r;

function [31:0] icache_addr_map;
    input [31:0] addr;
    reg   [31:0] dmw_fixed_addr;
    begin
        // Keep the exp19 fix for instruction fetch through DMW0/DMW1.
        dmw_fixed_addr = (addr[31:29] == 3'b111) ? {3'b110, addr[28:0]} : addr;

        if (dmw_fixed_addr[31:16] == 16'hbfaf || dmw_fixed_addr[31:16] == 16'h1faf) begin
            icache_addr_map = dmw_fixed_addr;
        end
        else if (dmw_fixed_addr[31:30] == 2'b10) begin
            icache_addr_map = {2'b00, dmw_fixed_addr[29:0]};
        end
        else begin
            icache_addr_map = dmw_fixed_addr;
        end
    end
endfunction

wire [31:0] inst_cache_paddr;
wire [31:0] data_cache_paddr;
assign inst_cache_paddr = icache_addr_map(inst_sram_addr);
assign data_cache_paddr = dcache_req_addr_mux;
assign cacop_icache_paddr = icache_addr_map(icache_cacop_addr_mux);
// DCache requests already carry the ID-stage translated physical address.  Use
// the same physical tag for CACOP pre-clean; ICache CACOP keeps the virtual tag
// policy above because IF still presents virtual PCs to the ICache wrapper.
assign cacop_dcache_paddr = icacop_dclean_valid ? icacop_paddr_r : dcacop_paddr_r;

`ifdef DISABLE_M174I_4WAY_ICACHE
localparam ICACHE_FOUR_WAY = 1'b0;
`else
localparam ICACHE_FOUR_WAY = 1'b1;
`endif
`ifdef DISABLE_M183_FIVE_LINE_FILL
localparam CACHE_FIVE_LINE_FILL = 1'b0;
`else
localparam CACHE_FIVE_LINE_FILL = 1'b1;
`endif
`ifdef DISABLE_M183_QUAD_ICACHE_FILL
localparam ICACHE_FOUR_LINE_FILL = 1'b0;
`else
localparam ICACHE_FOUR_LINE_FILL = 1'b1;
`endif
`ifdef DISABLE_M180_ICACHE_LINE_FILL
localparam ICACHE_ADJACENT_LINE_FILL = 1'b0;
`else
localparam ICACHE_ADJACENT_LINE_FILL = 1'b1;
`endif

cache #(.PIPELINED_HIT(1'b1),
        .FOUR_WAY(ICACHE_FOUR_WAY),
        .SETS_512(1'b0),
        .SETS_1024(1'b0),
        .ADJACENT_LINE_FILL(ICACHE_ADJACENT_LINE_FILL),
        .FOUR_LINE_FILL(ICACHE_FOUR_LINE_FILL),
        .FIVE_LINE_FILL(CACHE_FIVE_LINE_FILL),
        .READ_ONLY_CACHE(1'b1)) u_icache(
    .clk       (clk                 ),
    .resetn    (resetn              ),

    .valid     (inst_sram_req       ),
    .op        (1'b0                ),
    .index     ({2'b0, inst_sram_addr[11:4]}),
    .tag       (inst_cache_paddr[31:12]),
    .offset    (inst_sram_addr[3:0] ),
    .wstrb     (4'b0000             ),
    .wdata     (32'b0               ),
    .dual_read (inst_sram_dual      ),
    .cacop_valid(cacop_to_icache     ),
    .cacop_code (icache_cacop_code_mux),
    .cacop_index({2'b0, icache_cacop_addr_mux[11:4]}),
    .cacop_tag  (cacop_icache_paddr[31:12]),
    .cacop_way  (icache_cacop_addr_mux[1:0]),
    .cacop_addr_ok(icache_cacop_addr_ok),
    .cacop_data_ok(icache_cacop_data_ok),
    .addr_ok   (inst_sram_addr_ok   ),
    .data_ok   (inst_sram_data_ok   ),
    .rdata     (inst_sram_rdata     ),
    .rdata2    (inst_sram_rdata2    ),

    .rd_req    (icache_rd_req       ),
    .rd_type   (icache_rd_type      ),
    .rd_addr   (icache_rd_addr      ),
    .rd_rdy    (icache_rd_rdy       ),
    .ret_valid (icache_ret_valid    ),
    .ret_last  (icache_ret_last     ),
    .ret_data  (icache_ret_data     ),

    .wr_req    (icache_wr_req       ),
    .wr_type   (icache_wr_type      ),
    .wr_addr   (icache_wr_addr      ),
    .wr_wstrb  (icache_wr_wstrb     ),
    .wr_data   (icache_wr_data      ),
    .wr_rdy    (icache_wr_rdy       )
);

`ifdef DISABLE_M170C_4WAY_DCACHE
localparam DCACHE_FOUR_WAY = 1'b0;
`else
localparam DCACHE_FOUR_WAY = 1'b1;
`endif
`ifdef DISABLE_M175D_32KB_DCACHE
localparam DCACHE_SETS_512 = 1'b0;
`else
`ifdef DISABLE_M170C_4WAY_DCACHE
// The legacy 2-way rollback remains the original 8KB/256-set geometry.
localparam DCACHE_SETS_512 = 1'b0;
`else
localparam DCACHE_SETS_512 = 1'b1;
`endif
`endif
`ifdef DISABLE_M184_64KB_DCACHE
localparam DCACHE_SETS_1024 = 1'b0;
`else
`ifdef DISABLE_M175D_32KB_DCACHE
localparam DCACHE_SETS_1024 = 1'b0;
`else
localparam DCACHE_SETS_1024 = 1'b1;
`endif
`endif
`ifdef DISABLE_M173N_HIT_UNDER_REFILL
localparam DCACHE_HIT_UNDER_REFILL = 1'b0;
`else
localparam DCACHE_HIT_UNDER_REFILL = 1'b1;
`endif
`ifdef DISABLE_M180B_ADJACENT_LINE_FILL
localparam DCACHE_ADJACENT_LINE_FILL = 1'b0;
`else
localparam DCACHE_ADJACENT_LINE_FILL = 1'b1;
`endif
`ifdef DISABLE_M186_PIPELINED_DCACHE
localparam DCACHE_PIPELINED_HIT = 1'b0;
`else
localparam DCACHE_PIPELINED_HIT = 1'b1;
`endif

wire [9:0] dcache_set_index = DCACHE_SETS_1024 ?
                                    dcache_req_addr_mux[13:4] :
                              DCACHE_SETS_512 ?
                                    {1'b0, dcache_req_addr_mux[12:4]} :
                                    {2'b0, dcache_req_addr_mux[11:4]};
wire [19:0] dcache_line_tag = DCACHE_SETS_1024 ?
                              {2'b0, data_cache_paddr[31:14]} :
                              DCACHE_SETS_512 ?
                              {1'b0, data_cache_paddr[31:13]} :
                              data_cache_paddr[31:12];
// M17.5D adds physical index bit 12, which is outside the 4KB page offset.
// Use the translated physical CACOP address for the DCache set index to avoid
// selecting a virtual alias when the 32KB PIPT geometry is enabled.
wire [9:0] dcache_cacop_set_index = DCACHE_SETS_1024 ?
                                          cacop_dcache_paddr[13:4] :
                                    DCACHE_SETS_512 ?
                                          {1'b0, cacop_dcache_paddr[12:4]} :
                                          {2'b0, cacop_dcache_paddr[11:4]};
wire [19:0] dcache_cacop_line_tag = DCACHE_SETS_1024 ?
                                    {2'b0, cacop_dcache_paddr[31:14]} :
                                    DCACHE_SETS_512 ?
                                    {1'b0, cacop_dcache_paddr[31:13]} :
                                    cacop_dcache_paddr[31:12];

cache #(.PIPELINED_HIT(DCACHE_PIPELINED_HIT),
        .FOUR_WAY(DCACHE_FOUR_WAY),
        .SETS_512(DCACHE_SETS_512),
        .SETS_1024(DCACHE_SETS_1024),
        .HIT_UNDER_REFILL(DCACHE_HIT_UNDER_REFILL),
        .ADJACENT_LINE_FILL(DCACHE_ADJACENT_LINE_FILL),
        // Fill four contiguous 16-byte DCache lines in one legal 16-beat AXI3
        // burst.  cache.v keeps the speculative-line address calculation in
        // cache-line units so this no longer recreates the old 32-bit carry
        // chain on the BRAM write path.
        .FOUR_LINE_FILL(DCACHE_ADJACENT_LINE_FILL),
        .FIVE_LINE_FILL(CACHE_FIVE_LINE_FILL &&
                        DCACHE_ADJACENT_LINE_FILL)) u_dcache(
    .clk       (clk                  ),
    .resetn    (resetn               ),

    .valid     (dcache_req_to_cache_valid),
    .op        (dcache_req_op_mux       ),
    .index     (dcache_set_index       ),
    .tag       (dcache_line_tag        ),
    .offset    (dcache_req_addr_mux[3:0]),
    .wstrb     (dcache_req_wstrb_mux    ),
    .wdata     (dcache_req_wdata_mux    ),
    .dual_read (1'b0                    ),
    .cacop_valid(cacop_to_dcache      ),
    .cacop_code (dcache_cacop_code_mux),
    .cacop_index(dcache_cacop_set_index),
    .cacop_tag  (dcache_cacop_line_tag ),
    .cacop_way  (dcache_cacop_addr_mux[1:0]),
    .cacop_addr_ok(dcache_cacop_addr_ok),
    .cacop_data_ok(dcache_cacop_data_ok),
    .addr_ok   (dcache_cpu_addr_ok   ),
    .data_ok   (dcache_cpu_data_ok   ),
    .rdata     (dcache_cpu_rdata     ),
    .rdata2    (                       ),

    .rd_req    (dcache_rd_req        ),
    .rd_type   (dcache_rd_type       ),
    .rd_addr   (dcache_rd_addr       ),
    .rd_rdy    (dcache_rd_rdy        ),
    .ret_valid (dcache_ret_valid     ),
    .ret_last  (dcache_ret_last      ),
    .ret_data  (dcache_ret_data      ),

    .wr_req    (dcache_wr_req        ),
    .wr_type   (dcache_wr_type       ),
    .wr_addr   (dcache_wr_addr       ),
    .wr_wstrb  (dcache_wr_wstrb      ),
    .wr_data   (dcache_wr_data       ),
    .wr_rdy    (dcache_wr_rdy        )
);


// -----------------------------------------------------------------------------
// Benchmark-window detection for performance counters.
//
// The NSCSCC performance tests clear the SoC timer, print "test begin", then
// read CONFREG_TIMER_BASE once to take start_count.  After the benchmark body,
// they read the same timer again to take stop_count.  Counting only between
// these two timer reads keeps startup .data/BSS initialization, UART printing,
// and final reporting out of the performance counters.
//
// DMW normally maps 0xbfaf_e000 to 0x1faf_e000, but both forms are accepted here
// to make the debug trigger robust across direct/DMW configurations.
// -----------------------------------------------------------------------------
`ifndef DISABLE_PERF_COUNTERS
wire perf_timer_read_addr_hit = (data_sram_addr[31:12] == 20'h1fafe) ||
                                (data_sram_addr[31:12] == 20'hbfafe);
wire perf_timer_read_done = data_uncache_req && !data_sram_wr &&
                            perf_timer_read_addr_hit && uncache_data_data_ok;

(* keep = "true", mark_debug = "true" *) reg        perf_count_active;
(* keep = "true", mark_debug = "true" *) reg        perf_count_start_pulse;
(* keep = "true", mark_debug = "true" *) reg        perf_count_stop_pulse;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_window_start_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_window_stop_cnt;

always @(posedge clk) begin
    if (!resetn) begin
        perf_count_active      <= 1'b0;
        perf_count_start_pulse <= 1'b0;
        perf_count_stop_pulse  <= 1'b0;
        perf_window_start_cnt  <= 32'b0;
        perf_window_stop_cnt   <= 32'b0;
    end else begin
        perf_count_start_pulse <= 1'b0;
        perf_count_stop_pulse  <= 1'b0;

        if (perf_timer_read_done) begin
            if (!perf_count_active) begin
                perf_count_active      <= 1'b1;
                perf_count_start_pulse <= 1'b1;
                perf_window_start_cnt  <= perf_window_start_cnt + 32'd1;
            end else begin
                perf_count_active      <= 1'b0;
                perf_count_stop_pulse  <= 1'b1;
                perf_window_stop_cnt   <= perf_window_stop_cnt + 32'd1;
            end
        end
    end
end
`else
wire perf_count_active;
wire perf_count_start_pulse;
assign perf_count_active = 1'b1;
assign perf_count_start_pulse = 1'b0;
`endif

// -----------------------------------------------------------------------------
// Performance counters, wrapper/cache/AXI side.
// These counters are intentionally not connected to the architectural ISA.  They
// are for Verilator/Vivado waveform inspection and for guiding the next round of
// optimization.  Disable with -DDISABLE_PERF_COUNTERS if needed.
// -----------------------------------------------------------------------------
`ifndef DISABLE_PERF_COUNTERS
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_top_cycle_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_if_req_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_if_wait_addr_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_if_wait_data_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_icache_miss_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_icache_refill_beat_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_dcache_req_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_dcache_rd_miss_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_dcache_wb_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_dcache_refill_beat_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_uncache_req_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_uncache_wait_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_axi_ar_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_axi_r_beat_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_axi_aw_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_axi_w_beat_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_axi_b_cnt;

always @(posedge clk) begin
    if (!resetn || perf_count_start_pulse) begin
        perf_top_cycle_cnt          <= 32'b0;
        perf_if_req_cnt             <= 32'b0;
        perf_if_wait_addr_cnt       <= 32'b0;
        perf_if_wait_data_cnt       <= 32'b0;
        perf_icache_miss_cnt        <= 32'b0;
        perf_icache_refill_beat_cnt <= 32'b0;
        perf_dcache_req_cnt         <= 32'b0;
        perf_dcache_rd_miss_cnt     <= 32'b0;
        perf_dcache_wb_cnt          <= 32'b0;
        perf_dcache_refill_beat_cnt <= 32'b0;
        perf_uncache_req_cnt        <= 32'b0;
        perf_uncache_wait_cnt       <= 32'b0;
        perf_axi_ar_cnt             <= 32'b0;
        perf_axi_r_beat_cnt         <= 32'b0;
        perf_axi_aw_cnt             <= 32'b0;
        perf_axi_w_beat_cnt         <= 32'b0;
        perf_axi_b_cnt              <= 32'b0;
    end else if (perf_count_active) begin
        perf_top_cycle_cnt <= perf_top_cycle_cnt + 32'd1;

        if (inst_sram_req && inst_sram_addr_ok)
            perf_if_req_cnt <= perf_if_req_cnt + 32'd1;
        if (inst_sram_req && !inst_sram_addr_ok)
            perf_if_wait_addr_cnt <= perf_if_wait_addr_cnt + 32'd1;
        if (inst_sram_req && inst_sram_addr_ok && !inst_sram_data_ok)
            perf_if_wait_data_cnt <= perf_if_wait_data_cnt + 32'd1;

        // ICache miss/refill events are counted at the cache memory-side port.
        if (icache_rd_req && icache_rd_rdy)
            perf_icache_miss_cnt <= perf_icache_miss_cnt + 32'd1;
        if (icache_ret_valid)
            perf_icache_refill_beat_cnt <= perf_icache_refill_beat_cnt + 32'd1;

        if (data_cache_req && data_sram_addr_ok)
            perf_dcache_req_cnt <= perf_dcache_req_cnt + 32'd1;
        if (dcache_rd_req && dcache_rd_rdy)
            perf_dcache_rd_miss_cnt <= perf_dcache_rd_miss_cnt + 32'd1;
        if (dcache_wr_req && dcache_wr_rdy)
            perf_dcache_wb_cnt <= perf_dcache_wb_cnt + 32'd1;
        if (dcache_ret_valid)
            perf_dcache_refill_beat_cnt <= perf_dcache_refill_beat_cnt + 32'd1;

        if (data_uncache_req && uncache_data_addr_ok)
            perf_uncache_req_cnt <= perf_uncache_req_cnt + 32'd1;
        if (data_uncache_req && !uncache_data_data_ok)
            perf_uncache_wait_cnt <= perf_uncache_wait_cnt + 32'd1;

        if (arvalid && arready)
            perf_axi_ar_cnt <= perf_axi_ar_cnt + 32'd1;
        if (rvalid && rready)
            perf_axi_r_beat_cnt <= perf_axi_r_beat_cnt + 32'd1;
        if (awvalid && awready)
            perf_axi_aw_cnt <= perf_axi_aw_cnt + 32'd1;
        if (wvalid && wready)
            perf_axi_w_beat_cnt <= perf_axi_w_beat_cnt + 32'd1;
        if (bvalid && bready)
            perf_axi_b_cnt <= perf_axi_b_cnt + 32'd1;
    end
end
`endif

mycpu_core u_mycpu_core(
    .clk              (clk              ),
    .resetn           (resetn           ),
    .ext_int          (8'b0             ),
    .perf_count_enable_i(perf_count_active),
    .perf_count_clear_i (perf_count_start_pulse),

    .inst_sram_req    (inst_sram_req    ),
    .inst_sram_wr     (inst_sram_wr     ),
    .inst_sram_size   (inst_sram_size   ),
    .inst_sram_wstrb  (inst_sram_wstrb  ),
    .inst_sram_addr   (inst_sram_addr   ),
    .inst_sram_wdata  (inst_sram_wdata  ),
    .inst_sram_dual   (inst_sram_dual   ),
    .inst_sram_addr_ok(inst_sram_addr_ok),
    .inst_sram_data_ok(inst_sram_data_ok),
    .inst_sram_rdata  (inst_sram_rdata  ),
    .inst_sram_rdata2 (inst_sram_rdata2 ),

    .data_sram_req    (data_sram_req    ),
    .data_sram_wr     (data_sram_wr     ),
    .data_sram_size   (data_sram_size   ),
    .data_sram_wstrb  (data_sram_wstrb  ),
    .data_sram_addr   (data_sram_addr   ),
    .data_sram_addr_fast(data_sram_addr_fast),
    .data_sram_addr_late(data_sram_addr_late),
    .data_sram_wdata  (data_sram_wdata  ),
    .data_sram_uncached(data_sram_uncached),
    .data_sram_addr_ok(data_sram_addr_ok),
    .data_sram_data_ok(data_sram_data_ok),
    .data_sram_rdata  (data_sram_rdata  ),
    .cacop_req        (cacop_req        ),
    .cacop_code       (cacop_code       ),
    .cacop_addr       (cacop_addr       ),
    .cacop_paddr      (cacop_paddr      ),
    .cacop_addr_ok    (cacop_addr_ok    ),
    .cacop_data_ok    (cacop_data_ok    ),

    .debug_wb_pc      (debug_wb_pc      ),
    .debug_wb_rf_we   (debug_wb_rf_we   ),
    .debug_wb_rf_wnum (debug_wb_rf_wnum ),
    .debug_wb_rf_wdata(debug_wb_rf_wdata)
);

sram_axi_2x1_bridge u_sram_axi_2x1_bridge(
    .clk              (clk              ),
    .resetn           (resetn           ),

    .icache_rd_req     (icache_rd_req     ),
    .icache_rd_type    (icache_rd_type    ),
    .icache_rd_addr    (icache_rd_addr    ),
    .icache_rd_rdy     (icache_rd_rdy     ),
    .icache_ret_valid  (icache_ret_valid  ),
    .icache_ret_last   (icache_ret_last   ),
    .icache_ret_data   (icache_ret_data   ),

    .icache_wr_req     (icache_wr_req     ),
    .icache_wr_type    (icache_wr_type    ),
    .icache_wr_addr    (icache_wr_addr    ),
    .icache_wr_wstrb   (icache_wr_wstrb   ),
    .icache_wr_data    (icache_wr_data    ),
    .icache_wr_rdy     (icache_wr_rdy     ),

    .dcache_rd_req     (dcache_rd_req     ),
    .dcache_rd_type    (dcache_rd_type    ),
    .dcache_rd_addr    (dcache_rd_addr    ),
    .dcache_rd_rdy     (dcache_rd_rdy     ),
    .dcache_ret_valid  (dcache_ret_valid  ),
    .dcache_ret_last   (dcache_ret_last   ),
    .dcache_ret_data   (dcache_ret_data   ),

    .dcache_wr_req     (dcache_wr_req     ),
    .dcache_wr_type    (dcache_wr_type    ),
    .dcache_wr_addr    (dcache_wr_addr    ),
    .dcache_wr_wstrb   (dcache_wr_wstrb   ),
    .dcache_wr_data    (dcache_wr_data    ),
    .dcache_wr_rdy     (dcache_wr_rdy     ),

    .uncache_req       (data_uncache_req  ),
    .uncache_wr        (data_sram_wr      ),
    .uncache_size      (data_sram_size    ),
    .uncache_wstrb     (data_sram_wstrb   ),
    .uncache_addr      (data_sram_addr    ),
    .uncache_wdata     (data_sram_wdata   ),
    .uncache_addr_ok   (uncache_data_addr_ok),
    .uncache_data_ok   (uncache_data_data_ok),
    .uncache_rdata     (uncache_data_rdata),

    .arid             (arid             ),
    .araddr           (araddr           ),
    .arlen            (arlen            ),
    .arsize           (arsize           ),
    .arburst          (arburst          ),
    .arlock           (arlock           ),
    .arcache          (arcache          ),
    .arprot           (arprot           ),
    .arvalid          (arvalid          ),
    .arready          (arready          ),
    .rid              (rid              ),
    .rdata            (rdata            ),
    .rresp            (rresp            ),
    .rlast            (rlast            ),
    .rvalid           (rvalid           ),
    .rready           (rready           ),
    .awid             (awid             ),
    .awaddr           (awaddr           ),
    .awlen            (awlen            ),
    .awsize           (awsize           ),
    .awburst          (awburst          ),
    .awlock           (awlock           ),
    .awcache          (awcache          ),
    .awprot           (awprot           ),
    .awvalid          (awvalid          ),
    .awready          (awready          ),
    .wid              (wid              ),
    .wdata            (wdata            ),
    .wstrb            (wstrb            ),
    .wlast            (wlast            ),
    .wvalid           (wvalid           ),
    .wready           (wready           ),
    .bid              (bid              ),
    .bresp            (bresp            ),
    .bvalid           (bvalid           ),
    .bready           (bready           )
);


// -----------------------------------------------------------------------------
// M16.0F simulation-only performance diagnosis.
//
// The official NSCSCC benchmark reads the SoC timer once before and once after
// the measured region.  perf_count_start_pulse / perf_count_stop_pulse use
// those two reads as an exact measurement window.  This block is excluded from
// synthesis, so it has no routed-area, WNS, or board-performance cost.
//
// Every completed benchmark prints machine-readable PERF_DIAG lines directly
// in the Vivado/XSim Tcl console.  Copy the complete BEGIN..END block when
// reporting results.
// -----------------------------------------------------------------------------
`ifdef ENABLE_SIM_PERF_DIAG

reg [7:0] diag_q_from_dual_second;
reg       diag_ds_from_dual_second;

reg [31:0] diag_fetch_single_req_cnt;
reg [31:0] diag_fetch_dual_req_cnt;
reg [31:0] diag_fetch_pred_taken_cnt;
reg [31:0] diag_ifq_empty_cycle_cnt;
reg [31:0] diag_ifq_one_cycle_cnt;
reg [31:0] diag_ifq_two_cycle_cnt;
reg [31:0] diag_ifq_three_cycle_cnt;
reg [31:0] diag_ifq_full_cycle_cnt;
reg [31:0] diag_ifq_head_unready_cycle_cnt;
reg [31:0] diag_fetch_pending_cycle_cnt;
reg [31:0] diag_frontend_starve_cycle_cnt;
reg [31:0] diag_redirect_cnt;
reg [31:0] diag_redirect_discard_entry_cnt;

reg [31:0] diag_main_issue_cnt;
reg [31:0] diag_pair_candidate_cnt;
reg [31:0] diag_pair_normal_cnt;
reg [31:0] diag_pair_swap_cnt;
reg [31:0] diag_pair_swap_mem_cnt;
reg [31:0] diag_pair_swap_mul_cnt;
reg [31:0] diag_pair_fail_no_next_cnt;
reg [31:0] diag_pair_fail_boundary_cnt;
reg [31:0] diag_pair_fail_class_cnt;
reg [31:0] diag_pair_fail_class_mem_cnt;
reg [31:0] diag_pair_fail_class_branch_cnt;
reg [31:0] diag_pair_fail_class_mul_cnt;
reg [31:0] diag_pair_fail_class_div_cnt;
reg [31:0] diag_pair_fail_class_system_cnt;
reg [31:0] diag_pair_fail_class_slot0_cnt;
reg [31:0] diag_pair_fail_class_other_cnt;
reg [31:0] diag_pair_fail_dependency_cnt;
reg [31:0] diag_dep_pair_raw_cnt;
reg [31:0] diag_dep_pair_waw_cnt;
reg [31:0] diag_dep_main_exe_cnt;
reg [31:0] diag_dep_side_exe_cnt;
reg [31:0] diag_dep_mem_load_cnt;
reg [31:0] diag_pair_fail_addrmode_cnt;
reg [31:0] diag_pair_fail_other_cnt;
reg [31:0] diag_backend_block_cycle_cnt;

reg [31:0] diag_bp_direction_miss_cnt;
reg [31:0] diag_bp_target_miss_cnt;
reg [31:0] diag_bp_cond_miss_cnt;
reg [31:0] diag_bp_second_slot_ctrl_cnt;
reg [31:0] diag_bp_second_slot_taken_cnt;
reg [31:0] diag_bp_second_slot_miss_cnt;

// M17.0C retains the M16.3P branch and memory hotspot diagnostics.  These arrays exist only in
// RTL simulation (the whole block is excluded by ENABLE_SIM_PERF_DIAG during
// synthesis), so they do not affect the official netlist or WNS.
localparam integer DIAG_BRANCH_HOT_SLOTS = 16;
localparam integer DIAG_MEMORY_HOT_SLOTS = 8;

reg        diag_branch_hot_valid [0:DIAG_BRANCH_HOT_SLOTS-1];
reg [31:0] diag_branch_hot_pc    [0:DIAG_BRANCH_HOT_SLOTS-1];
reg [31:0] diag_branch_hot_exec  [0:DIAG_BRANCH_HOT_SLOTS-1];
reg [31:0] diag_branch_hot_miss  [0:DIAG_BRANCH_HOT_SLOTS-1];
reg [31:0] diag_branch_hot_taken [0:DIAG_BRANCH_HOT_SLOTS-1];
reg [31:0] diag_branch_hot_cond  [0:DIAG_BRANCH_HOT_SLOTS-1];
reg [31:0] diag_branch_hot_back  [0:DIAG_BRANCH_HOT_SLOTS-1];
reg [31:0] diag_branch_hot_flip  [0:DIAG_BRANCH_HOT_SLOTS-1];
reg        diag_branch_hot_last_taken [0:DIAG_BRANCH_HOT_SLOTS-1];

reg        diag_memory_hot_valid [0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_pc    [0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_wait  [0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_load  [0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_store [0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_lookup[0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_wb    [0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_rdreq [0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_refill[0:DIAG_MEMORY_HOT_SLOTS-1];
reg [31:0] diag_memory_hot_uncache[0:DIAG_MEMORY_HOT_SLOTS-1];

reg [31:0] diag_mem_load_req_cnt;
reg [31:0] diag_mem_store_req_cnt;
reg [31:0] diag_mem_load_hit_cnt;
reg [31:0] diag_mem_store_hit_cnt;
reg [31:0] diag_mem_load_miss_cnt;
reg [31:0] diag_mem_store_miss_cnt;
reg [31:0] diag_mem_stage_wait_cnt;
reg [31:0] diag_mem_lookup_wait_cnt;
reg [31:0] diag_mem_wb_wait_cnt;
reg [31:0] diag_mem_rdreq_wait_cnt;
reg [31:0] diag_mem_refill_wait_cnt;
reg [31:0] diag_mem_uncache_stage_wait_cnt;
reg [31:0] diag_mem_other_wait_cnt;
reg [31:0] diag_dcache_lookup_cycle_cnt;
reg [31:0] diag_dcache_wb_state_cycle_cnt;
reg [31:0] diag_dcache_rdreq_state_cycle_cnt;
reg [31:0] diag_dcache_refill_state_cycle_cnt;
reg [31:0] diag_dcache_refill_no_data_cycle_cnt;
reg [31:0] diag_dcache_wb_not_ready_cycle_cnt;
reg [31:0] diag_dcache_rd_not_ready_cycle_cnt;
reg [31:0] diag_dcache_way0_hit_cnt;
reg [31:0] diag_dcache_way1_hit_cnt;
reg [31:0] diag_dcache_way2_hit_cnt;
reg [31:0] diag_dcache_way3_hit_cnt;
reg [31:0] diag_dcache_way0_victim_cnt;
reg [31:0] diag_dcache_way1_victim_cnt;
reg [31:0] diag_dcache_way2_victim_cnt;
reg [31:0] diag_dcache_way3_victim_cnt;

integer diag_i;
integer diag_branch_hit_idx;
integer diag_branch_min_idx;
integer diag_memory_hit_idx;
integer diag_memory_min_idx;
reg [31:0] diag_branch_min_count;
reg [31:0] diag_memory_min_count;

wire diag_request_accept = u_mycpu_core.if_stage.request_accept;
wire diag_request_dual   = u_mycpu_core.if_stage.active_req_dual;
wire diag_request_pred_taken = u_mycpu_core.if_stage.active_req_pred_taken;
wire [3:0] diag_ifq_count = u_mycpu_core.if_stage.queue_count;
wire diag_ifq_head_ready = u_mycpu_core.if_stage.fs_to_ds_valid;
wire diag_redirect_fire  = u_mycpu_core.if_stage.br_taken;

wire diag_main_issue = u_mycpu_core.id_stage.ds_to_es_valid &&
                       u_mycpu_core.id_stage.es_allowin;
wire diag_pair_has_next = u_mycpu_core.id_stage.fs_to_ds_valid;
wire diag_pair_success = u_mycpu_core.id_stage.dual_pair_fire;
wire diag_pair_adjacent =
     (u_mycpu_core.id_stage.slot1_candidate_pc ==
      (u_mycpu_core.id_stage.ds_pc + 32'd4));
wire diag_pair_same_page =
     (u_mycpu_core.id_stage.slot1_candidate_pc[31:12] ==
      u_mycpu_core.id_stage.ds_pc[31:12]);
wire diag_normal_shape = u_mycpu_core.id_stage.slot0_dual_eligible &&
                         u_mycpu_core.id_stage.slot1_candidate_simple;
wire diag_swap_mem_shape = u_mycpu_core.id_stage.current_simple_alu &&
                           u_mycpu_core.id_stage.slot1_candidate_mem;
wire diag_swap_mul_shape = u_mycpu_core.id_stage.current_simple_alu &&
                           u_mycpu_core.id_stage.slot1_candidate_mul;
wire diag_swap_shape = diag_swap_mem_shape | diag_swap_mul_shape;
wire diag_normal_dependency =
       u_mycpu_core.id_stage.slot1_src1_es_hazard ||
       u_mycpu_core.id_stage.slot1_src2_es_hazard ||
       u_mycpu_core.id_stage.slot1_src1_ms_load_hazard ||
       u_mycpu_core.id_stage.slot1_src2_ms_load_hazard ||
       u_mycpu_core.id_stage.slot1_pair_raw ||
       u_mycpu_core.id_stage.slot1_pair_waw;
wire diag_swap_mem_dependency =
       u_mycpu_core.id_stage.swap_candidate_es_load_hazard ||
       u_mycpu_core.id_stage.swap_candidate_side_es_hazard ||
       u_mycpu_core.id_stage.swap_candidate_ms_load_hazard ||
       u_mycpu_core.id_stage.swap_pair_raw ||
       u_mycpu_core.id_stage.swap_pair_waw;
wire diag_swap_mul_dependency =
       u_mycpu_core.id_stage.mul_swap_es_load_hazard ||
       u_mycpu_core.id_stage.mul_swap_side_es_hazard ||
       u_mycpu_core.id_stage.mul_swap_ms_load_hazard ||
       u_mycpu_core.id_stage.mul_swap_pair_raw ||
       u_mycpu_core.id_stage.mul_swap_pair_waw;
wire diag_swap_dependency = (diag_swap_mem_shape && diag_swap_mem_dependency) ||
                            (diag_swap_mul_shape && diag_swap_mul_dependency);
wire diag_swap_addrmode_fail = diag_swap_mem_shape &&
      (!u_mycpu_core.id_stage.swap_candidate_addrmode_ok ||
       !u_mycpu_core.id_stage.swap_candidate_aligned);
wire diag_dep_pair_raw = (diag_normal_shape && u_mycpu_core.id_stage.slot1_pair_raw) ||
                         (diag_swap_mem_shape && u_mycpu_core.id_stage.swap_pair_raw) ||
                         (diag_swap_mul_shape && u_mycpu_core.id_stage.mul_swap_pair_raw);
wire diag_dep_pair_waw = (diag_normal_shape && u_mycpu_core.id_stage.slot1_pair_waw) ||
                         (diag_swap_mem_shape && u_mycpu_core.id_stage.swap_pair_waw) ||
                         (diag_swap_mul_shape && u_mycpu_core.id_stage.mul_swap_pair_waw);
wire diag_dep_main_exe = (diag_normal_shape &&
                          (u_mycpu_core.id_stage.slot1_src1_es_hazard ||
                           u_mycpu_core.id_stage.slot1_src2_es_hazard)) ||
                         (diag_swap_mem_shape && u_mycpu_core.id_stage.swap_candidate_es_load_hazard) ||
                         (diag_swap_mul_shape && u_mycpu_core.id_stage.mul_swap_es_load_hazard);
wire diag_dep_side_exe = (diag_swap_mem_shape && u_mycpu_core.id_stage.swap_candidate_side_es_hazard) ||
                         (diag_swap_mul_shape && u_mycpu_core.id_stage.mul_swap_side_es_hazard);
wire diag_dep_mem_load = (diag_normal_shape &&
                          (u_mycpu_core.id_stage.slot1_src1_ms_load_hazard ||
                           u_mycpu_core.id_stage.slot1_src2_ms_load_hazard)) ||
                         (diag_swap_mem_shape && u_mycpu_core.id_stage.swap_candidate_ms_load_hazard) ||
                         (diag_swap_mul_shape && u_mycpu_core.id_stage.mul_swap_ms_load_hazard);
wire diag_candidate_branch = u_mycpu_core.id_stage.slot1_candidate_branch;
wire diag_candidate_mul = u_mycpu_core.id_stage.slot1_candidate_mul;
wire diag_candidate_div = u_mycpu_core.id_stage.slot1_candidate_div;
wire diag_candidate_system = u_mycpu_core.id_stage.slot1_candidate_system;
wire diag_candidate_mem = u_mycpu_core.id_stage.slot1_candidate_mem;
wire diag_candidate_simple = u_mycpu_core.id_stage.slot1_candidate_simple;
wire diag_slot0_class_block = diag_candidate_simple &&
                              !u_mycpu_core.id_stage.slot0_dual_eligible;
wire diag_pair_failed = diag_main_issue && diag_pair_has_next &&
                        !diag_pair_success;

wire diag_branch_update = u_mycpu_core.bp_update_en;
wire [31:0] diag_branch_pc = u_mycpu_core.bp_update_pc;
wire diag_branch_cond = u_mycpu_core.bp_update_is_cond;
wire diag_branch_taken = u_mycpu_core.bp_update_taken;
wire [31:0] diag_branch_target = u_mycpu_core.bp_update_target;
wire diag_branch_miss = u_mycpu_core.id_normal_redirect_valid |
                        u_mycpu_core.es_deferred_redirect_valid;
wire diag_branch_backward = (diag_branch_target < diag_branch_pc);

wire diag_mem_waiting = u_mycpu_core.mem_stage.ms_valid &&
                        u_mycpu_core.mem_stage.ms_mem_access &&
                        !u_mycpu_core.mem_stage.ms_ready_go &&
                        !u_mycpu_core.mem_stage.ms_inst_cacop;
wire [31:0] diag_mem_wait_pc = u_mycpu_core.mem_stage.ms_pc;
wire diag_mem_wait_load = |u_mycpu_core.mem_stage.ms_load_op;
wire diag_mem_wait_uncache = (u_sram_axi_2x1_bridge.req_src == 2'd3);
wire [2:0] diag_dcache_state = u_dcache.state;
wire diag_dcache_lookup = (diag_dcache_state == 3'd2);
wire diag_dcache_wb_state = (diag_dcache_state == 3'd3);
wire diag_dcache_rdreq_state = (diag_dcache_state == 3'd4);
wire diag_dcache_refill_state = (diag_dcache_state == 3'd5);
wire diag_dcache_lookup_hit = diag_dcache_lookup && u_dcache.cache_hit;
wire diag_dcache_lookup_miss = diag_dcache_lookup && !u_dcache.cache_hit;
wire diag_dcache_req_is_store = u_dcache.req_op;

function [63:0] diag_ratio_x1000;
    input [31:0] numerator;
    input [31:0] denominator;
    begin
        diag_ratio_x1000 = denominator ?
            (({32'b0, numerator} * 64'd1000) / denominator) : 64'd0;
    end
endfunction

function [63:0] diag_ratio_x10000;
    input [31:0] numerator;
    input [31:0] denominator;
    begin
        diag_ratio_x10000 = denominator ?
            (({32'b0, numerator} * 64'd10000) / denominator) : 64'd0;
    end
endfunction

// Shadow only the origin of each FIFO entry.  The functional IF FIFO and all
// architectural buses remain unchanged.  A branch carried as the second word
// of a dual-fetch bundle can therefore be diagnosed separately.
always @(posedge clk) begin
    if (!resetn || diag_redirect_fire) begin
        diag_q_from_dual_second <= 8'b0;
        diag_ds_from_dual_second <= 1'b0;
    end else begin
        if (diag_request_accept) begin
            diag_q_from_dual_second[u_mycpu_core.if_stage.tail_ptr] <= 1'b0;
            if (diag_request_dual)
                diag_q_from_dual_second[u_mycpu_core.if_stage.tail_ptr + 3'd1] <= 1'b1;
        end

        if (u_mycpu_core.id_stage.ds_allowin) begin
            if (u_mycpu_core.id_stage.dual_pair_fire &&
                u_mycpu_core.id_stage.fs_to_ds_valid1)
                diag_ds_from_dual_second <=
                    diag_q_from_dual_second[u_mycpu_core.if_stage.head_ptr + 3'd1];
            else if (u_mycpu_core.id_stage.fs_to_ds_valid)
                diag_ds_from_dual_second <=
                    diag_q_from_dual_second[u_mycpu_core.if_stage.head_ptr];
            else
                diag_ds_from_dual_second <= 1'b0;
        end
    end
end

always @(posedge clk) begin
    if (!resetn || perf_count_start_pulse) begin
        diag_fetch_single_req_cnt         <= 32'b0;
        diag_fetch_dual_req_cnt           <= 32'b0;
        diag_fetch_pred_taken_cnt         <= 32'b0;
        diag_ifq_empty_cycle_cnt          <= 32'b0;
        diag_ifq_one_cycle_cnt            <= 32'b0;
        diag_ifq_two_cycle_cnt            <= 32'b0;
        diag_ifq_three_cycle_cnt          <= 32'b0;
        diag_ifq_full_cycle_cnt           <= 32'b0;
        diag_ifq_head_unready_cycle_cnt   <= 32'b0;
        diag_fetch_pending_cycle_cnt      <= 32'b0;
        diag_frontend_starve_cycle_cnt    <= 32'b0;
        diag_redirect_cnt                 <= 32'b0;
        diag_redirect_discard_entry_cnt   <= 32'b0;
        diag_main_issue_cnt               <= 32'b0;
        diag_pair_candidate_cnt           <= 32'b0;
        diag_pair_normal_cnt              <= 32'b0;
        diag_pair_swap_cnt                <= 32'b0;
        diag_pair_swap_mem_cnt            <= 32'b0;
        diag_pair_swap_mul_cnt            <= 32'b0;
        diag_pair_fail_no_next_cnt        <= 32'b0;
        diag_pair_fail_boundary_cnt       <= 32'b0;
        diag_pair_fail_class_cnt          <= 32'b0;
        diag_pair_fail_class_mem_cnt      <= 32'b0;
        diag_pair_fail_class_branch_cnt   <= 32'b0;
        diag_pair_fail_class_mul_cnt      <= 32'b0;
        diag_pair_fail_class_div_cnt      <= 32'b0;
        diag_pair_fail_class_system_cnt   <= 32'b0;
        diag_pair_fail_class_slot0_cnt    <= 32'b0;
        diag_pair_fail_class_other_cnt    <= 32'b0;
        diag_pair_fail_dependency_cnt     <= 32'b0;
        diag_dep_pair_raw_cnt             <= 32'b0;
        diag_dep_pair_waw_cnt             <= 32'b0;
        diag_dep_main_exe_cnt             <= 32'b0;
        diag_dep_side_exe_cnt             <= 32'b0;
        diag_dep_mem_load_cnt             <= 32'b0;
        diag_pair_fail_addrmode_cnt       <= 32'b0;
        diag_pair_fail_other_cnt          <= 32'b0;
        diag_backend_block_cycle_cnt      <= 32'b0;
        diag_bp_direction_miss_cnt        <= 32'b0;
        diag_bp_target_miss_cnt           <= 32'b0;
        diag_bp_cond_miss_cnt             <= 32'b0;
        diag_bp_second_slot_ctrl_cnt      <= 32'b0;
        diag_bp_second_slot_taken_cnt     <= 32'b0;
        diag_bp_second_slot_miss_cnt      <= 32'b0;
        diag_mem_load_req_cnt              <= 32'b0;
        diag_mem_store_req_cnt             <= 32'b0;
        diag_mem_load_hit_cnt              <= 32'b0;
        diag_mem_store_hit_cnt             <= 32'b0;
        diag_mem_load_miss_cnt             <= 32'b0;
        diag_mem_store_miss_cnt            <= 32'b0;
        diag_mem_stage_wait_cnt            <= 32'b0;
        diag_mem_lookup_wait_cnt           <= 32'b0;
        diag_mem_wb_wait_cnt               <= 32'b0;
        diag_mem_rdreq_wait_cnt            <= 32'b0;
        diag_mem_refill_wait_cnt           <= 32'b0;
        diag_mem_uncache_stage_wait_cnt    <= 32'b0;
        diag_mem_other_wait_cnt            <= 32'b0;
        diag_dcache_lookup_cycle_cnt       <= 32'b0;
        diag_dcache_wb_state_cycle_cnt     <= 32'b0;
        diag_dcache_rdreq_state_cycle_cnt  <= 32'b0;
        diag_dcache_refill_state_cycle_cnt <= 32'b0;
        diag_dcache_refill_no_data_cycle_cnt <= 32'b0;
        diag_dcache_wb_not_ready_cycle_cnt <= 32'b0;
        diag_dcache_rd_not_ready_cycle_cnt <= 32'b0;
        diag_dcache_way0_hit_cnt           <= 32'b0;
        diag_dcache_way1_hit_cnt           <= 32'b0;
        diag_dcache_way2_hit_cnt           <= 32'b0;
        diag_dcache_way3_hit_cnt           <= 32'b0;
        diag_dcache_way0_victim_cnt        <= 32'b0;
        diag_dcache_way1_victim_cnt        <= 32'b0;
        diag_dcache_way2_victim_cnt        <= 32'b0;
        diag_dcache_way3_victim_cnt        <= 32'b0;
        for (diag_i = 0; diag_i < DIAG_BRANCH_HOT_SLOTS; diag_i = diag_i + 1) begin
            diag_branch_hot_valid[diag_i]      <= 1'b0;
            diag_branch_hot_pc[diag_i]         <= 32'b0;
            diag_branch_hot_exec[diag_i]       <= 32'b0;
            diag_branch_hot_miss[diag_i]       <= 32'b0;
            diag_branch_hot_taken[diag_i]      <= 32'b0;
            diag_branch_hot_cond[diag_i]       <= 32'b0;
            diag_branch_hot_back[diag_i]       <= 32'b0;
            diag_branch_hot_flip[diag_i]       <= 32'b0;
            diag_branch_hot_last_taken[diag_i] <= 1'b0;
        end
        for (diag_i = 0; diag_i < DIAG_MEMORY_HOT_SLOTS; diag_i = diag_i + 1) begin
            diag_memory_hot_valid[diag_i]   <= 1'b0;
            diag_memory_hot_pc[diag_i]      <= 32'b0;
            diag_memory_hot_wait[diag_i]    <= 32'b0;
            diag_memory_hot_load[diag_i]    <= 32'b0;
            diag_memory_hot_store[diag_i]   <= 32'b0;
            diag_memory_hot_lookup[diag_i]  <= 32'b0;
            diag_memory_hot_wb[diag_i]      <= 32'b0;
            diag_memory_hot_rdreq[diag_i]   <= 32'b0;
            diag_memory_hot_refill[diag_i]  <= 32'b0;
            diag_memory_hot_uncache[diag_i] <= 32'b0;
        end
    end else if (perf_count_active) begin
        if (diag_request_accept && diag_request_dual)
            diag_fetch_dual_req_cnt <= diag_fetch_dual_req_cnt + 32'd1;
        if (diag_request_accept && !diag_request_dual)
            diag_fetch_single_req_cnt <= diag_fetch_single_req_cnt + 32'd1;
        if (diag_request_accept && diag_request_pred_taken)
            diag_fetch_pred_taken_cnt <= diag_fetch_pred_taken_cnt + 32'd1;

        case (diag_ifq_count)
            4'd0: diag_ifq_empty_cycle_cnt <= diag_ifq_empty_cycle_cnt + 32'd1;
            4'd1: diag_ifq_one_cycle_cnt   <= diag_ifq_one_cycle_cnt + 32'd1;
            4'd2: diag_ifq_two_cycle_cnt   <= diag_ifq_two_cycle_cnt + 32'd1;
            4'd3: diag_ifq_three_cycle_cnt <= diag_ifq_three_cycle_cnt + 32'd1;
            default: diag_ifq_full_cycle_cnt <= diag_ifq_full_cycle_cnt + 32'd1;
        endcase
        if ((diag_ifq_count != 4'd0) && !diag_ifq_head_ready)
            diag_ifq_head_unready_cycle_cnt <= diag_ifq_head_unready_cycle_cnt + 32'd1;
        if (u_mycpu_core.if_stage.req_pending)
            diag_fetch_pending_cycle_cnt <= diag_fetch_pending_cycle_cnt + 32'd1;
        if (!u_mycpu_core.id_stage.ds_valid && u_mycpu_core.id_stage.es_allowin)
            diag_frontend_starve_cycle_cnt <= diag_frontend_starve_cycle_cnt + 32'd1;
        if (diag_redirect_fire) begin
            diag_redirect_cnt <= diag_redirect_cnt + 32'd1;
            diag_redirect_discard_entry_cnt <= diag_redirect_discard_entry_cnt +
                                               {29'b0, diag_ifq_count};
        end

        if (diag_main_issue)
            diag_main_issue_cnt <= diag_main_issue_cnt + 32'd1;
        if (diag_main_issue && diag_pair_has_next)
            diag_pair_candidate_cnt <= diag_pair_candidate_cnt + 32'd1;
        if (u_mycpu_core.id_stage.pair_normal_fire)
            diag_pair_normal_cnt <= diag_pair_normal_cnt + 32'd1;
        if (u_mycpu_core.id_stage.pair_swap_fire)
            diag_pair_swap_cnt <= diag_pair_swap_cnt + 32'd1;
        if (u_mycpu_core.id_stage.pair_swap_mem_fire)
            diag_pair_swap_mem_cnt <= diag_pair_swap_mem_cnt + 32'd1;
        if (u_mycpu_core.id_stage.pair_swap_mul_fire)
            diag_pair_swap_mul_cnt <= diag_pair_swap_mul_cnt + 32'd1;
        if (diag_main_issue && !diag_pair_has_next)
            diag_pair_fail_no_next_cnt <= diag_pair_fail_no_next_cnt + 32'd1;
        if (diag_pair_failed) begin
            if (!diag_pair_adjacent || !diag_pair_same_page)
                diag_pair_fail_boundary_cnt <= diag_pair_fail_boundary_cnt + 32'd1;
            else if (!diag_normal_shape && !diag_swap_shape) begin
                diag_pair_fail_class_cnt <= diag_pair_fail_class_cnt + 32'd1;
                if (diag_candidate_mem)
                    diag_pair_fail_class_mem_cnt <= diag_pair_fail_class_mem_cnt + 32'd1;
                else if (diag_candidate_branch)
                    diag_pair_fail_class_branch_cnt <= diag_pair_fail_class_branch_cnt + 32'd1;
                else if (diag_candidate_mul)
                    diag_pair_fail_class_mul_cnt <= diag_pair_fail_class_mul_cnt + 32'd1;
                else if (diag_candidate_div)
                    diag_pair_fail_class_div_cnt <= diag_pair_fail_class_div_cnt + 32'd1;
                else if (diag_candidate_system)
                    diag_pair_fail_class_system_cnt <= diag_pair_fail_class_system_cnt + 32'd1;
                else if (diag_slot0_class_block)
                    diag_pair_fail_class_slot0_cnt <= diag_pair_fail_class_slot0_cnt + 32'd1;
                else
                    diag_pair_fail_class_other_cnt <= diag_pair_fail_class_other_cnt + 32'd1;
            end else if (diag_swap_addrmode_fail)
                diag_pair_fail_addrmode_cnt <= diag_pair_fail_addrmode_cnt + 32'd1;
            else if ((diag_normal_shape && diag_normal_dependency) ||
                     (diag_swap_shape && diag_swap_dependency)) begin
                diag_pair_fail_dependency_cnt <= diag_pair_fail_dependency_cnt + 32'd1;
                if (diag_dep_pair_raw)
                    diag_dep_pair_raw_cnt <= diag_dep_pair_raw_cnt + 32'd1;
                if (diag_dep_pair_waw)
                    diag_dep_pair_waw_cnt <= diag_dep_pair_waw_cnt + 32'd1;
                if (diag_dep_main_exe)
                    diag_dep_main_exe_cnt <= diag_dep_main_exe_cnt + 32'd1;
                if (diag_dep_side_exe)
                    diag_dep_side_exe_cnt <= diag_dep_side_exe_cnt + 32'd1;
                if (diag_dep_mem_load)
                    diag_dep_mem_load_cnt <= diag_dep_mem_load_cnt + 32'd1;
            end
            else
                diag_pair_fail_other_cnt <= diag_pair_fail_other_cnt + 32'd1;
        end
        if (u_mycpu_core.id_stage.ds_valid &&
            u_mycpu_core.id_stage.ds_ready_go &&
            !u_mycpu_core.id_stage.es_allowin)
            diag_backend_block_cycle_cnt <= diag_backend_block_cycle_cnt + 32'd1;

        if ((u_mycpu_core.id_stage.bp_update_en &&
             u_mycpu_core.id_stage.bp_direction_miss) ||
            (u_mycpu_core.exe_stage.deferred_resolve_fire &&
             (u_mycpu_core.exe_stage.es_deferred_pred_taken ^
              u_mycpu_core.exe_stage.deferred_taken_resolved)))
            diag_bp_direction_miss_cnt <= diag_bp_direction_miss_cnt + 32'd1;
        if ((u_mycpu_core.id_stage.bp_update_en &&
             u_mycpu_core.id_stage.bp_target_miss) ||
             (u_mycpu_core.exe_stage.deferred_resolve_fire &&
              u_mycpu_core.exe_stage.deferred_taken_resolved &&
              u_mycpu_core.exe_stage.deferred_pred_target_miss_resolved))
            diag_bp_target_miss_cnt <= diag_bp_target_miss_cnt + 32'd1;
        if ((u_mycpu_core.id_stage.bp_update_en &&
             u_mycpu_core.id_stage.bp_update_is_cond &&
             u_mycpu_core.id_stage.id_normal_redirect_valid) ||
            u_mycpu_core.es_deferred_redirect_valid)
            diag_bp_cond_miss_cnt <= diag_bp_cond_miss_cnt + 32'd1;
        if (u_mycpu_core.id_stage.bp_update_en && diag_ds_from_dual_second) begin
            diag_bp_second_slot_ctrl_cnt <= diag_bp_second_slot_ctrl_cnt + 32'd1;
            if (u_mycpu_core.id_stage.bp_update_taken)
                diag_bp_second_slot_taken_cnt <= diag_bp_second_slot_taken_cnt + 32'd1;
            if (u_mycpu_core.id_stage.id_normal_redirect_valid)
                diag_bp_second_slot_miss_cnt <= diag_bp_second_slot_miss_cnt + 32'd1;
        end

        // Track the hottest dynamic branch PCs.  A 16-entry space-saving table
        // keeps the console output compact while preserving the dominant PCs.
        if (diag_branch_update) begin
            diag_branch_hit_idx = -1;
            diag_branch_min_idx = 0;
            diag_branch_min_count = 32'hffff_ffff;
            for (diag_i = 0; diag_i < DIAG_BRANCH_HOT_SLOTS; diag_i = diag_i + 1) begin
                if (diag_branch_hot_valid[diag_i] &&
                    (diag_branch_hot_pc[diag_i] == diag_branch_pc))
                    diag_branch_hit_idx = diag_i;
                if (!diag_branch_hot_valid[diag_i]) begin
                    diag_branch_min_idx = diag_i;
                    diag_branch_min_count = 32'b0;
                end else if (diag_branch_hot_exec[diag_i] < diag_branch_min_count) begin
                    diag_branch_min_idx = diag_i;
                    diag_branch_min_count = diag_branch_hot_exec[diag_i];
                end
            end
            if (diag_branch_hit_idx >= 0) begin
                diag_branch_hot_exec[diag_branch_hit_idx] <= diag_branch_hot_exec[diag_branch_hit_idx] + 32'd1;
                if (diag_branch_miss)
                    diag_branch_hot_miss[diag_branch_hit_idx] <= diag_branch_hot_miss[diag_branch_hit_idx] + 32'd1;
                if (diag_branch_taken)
                    diag_branch_hot_taken[diag_branch_hit_idx] <= diag_branch_hot_taken[diag_branch_hit_idx] + 32'd1;
                if (diag_branch_cond)
                    diag_branch_hot_cond[diag_branch_hit_idx] <= diag_branch_hot_cond[diag_branch_hit_idx] + 32'd1;
                if (diag_branch_backward)
                    diag_branch_hot_back[diag_branch_hit_idx] <= diag_branch_hot_back[diag_branch_hit_idx] + 32'd1;
                if ((diag_branch_hot_exec[diag_branch_hit_idx] != 0) &&
                    (diag_branch_hot_last_taken[diag_branch_hit_idx] != diag_branch_taken))
                    diag_branch_hot_flip[diag_branch_hit_idx] <= diag_branch_hot_flip[diag_branch_hit_idx] + 32'd1;
                diag_branch_hot_last_taken[diag_branch_hit_idx] <= diag_branch_taken;
            end else begin
                diag_branch_hot_valid[diag_branch_min_idx]      <= 1'b1;
                diag_branch_hot_pc[diag_branch_min_idx]         <= diag_branch_pc;
                diag_branch_hot_exec[diag_branch_min_idx]       <= diag_branch_min_count + 32'd1;
                diag_branch_hot_miss[diag_branch_min_idx]       <= diag_branch_miss ? 32'd1 : 32'd0;
                diag_branch_hot_taken[diag_branch_min_idx]      <= diag_branch_taken ? 32'd1 : 32'd0;
                diag_branch_hot_cond[diag_branch_min_idx]       <= diag_branch_cond ? 32'd1 : 32'd0;
                diag_branch_hot_back[diag_branch_min_idx]       <= diag_branch_backward ? 32'd1 : 32'd0;
                diag_branch_hot_flip[diag_branch_min_idx]       <= 32'b0;
                diag_branch_hot_last_taken[diag_branch_min_idx] <= diag_branch_taken;
            end
        end

        // Cache request and state breakdown.  Request counts are taken at the
        // CPU/cache acceptance boundary; hit/miss type is classified in LOOKUP.
        if (data_cache_req_accept) begin
            if (data_sram_wr)
                diag_mem_store_req_cnt <= diag_mem_store_req_cnt + 32'd1;
            else
                diag_mem_load_req_cnt <= diag_mem_load_req_cnt + 32'd1;
        end
        if (diag_dcache_lookup)
            diag_dcache_lookup_cycle_cnt <= diag_dcache_lookup_cycle_cnt + 32'd1;
        if (diag_dcache_wb_state) begin
            diag_dcache_wb_state_cycle_cnt <= diag_dcache_wb_state_cycle_cnt + 32'd1;
            if (!dcache_wr_rdy)
                diag_dcache_wb_not_ready_cycle_cnt <= diag_dcache_wb_not_ready_cycle_cnt + 32'd1;
        end
        if (diag_dcache_rdreq_state) begin
            diag_dcache_rdreq_state_cycle_cnt <= diag_dcache_rdreq_state_cycle_cnt + 32'd1;
            if (!dcache_rd_rdy)
                diag_dcache_rd_not_ready_cycle_cnt <= diag_dcache_rd_not_ready_cycle_cnt + 32'd1;
        end
        if (diag_dcache_refill_state) begin
            diag_dcache_refill_state_cycle_cnt <= diag_dcache_refill_state_cycle_cnt + 32'd1;
            if (!dcache_ret_valid)
                diag_dcache_refill_no_data_cycle_cnt <= diag_dcache_refill_no_data_cycle_cnt + 32'd1;
        end
        if (diag_dcache_lookup_hit) begin
            if (u_dcache.way0_hit)
                diag_dcache_way0_hit_cnt <= diag_dcache_way0_hit_cnt + 32'd1;
            else if (u_dcache.way1_hit)
                diag_dcache_way1_hit_cnt <= diag_dcache_way1_hit_cnt + 32'd1;
            else if (u_dcache.way2_hit)
                diag_dcache_way2_hit_cnt <= diag_dcache_way2_hit_cnt + 32'd1;
            else if (u_dcache.way3_hit)
                diag_dcache_way3_hit_cnt <= diag_dcache_way3_hit_cnt + 32'd1;
            if (diag_dcache_req_is_store)
                diag_mem_store_hit_cnt <= diag_mem_store_hit_cnt + 32'd1;
            else
                diag_mem_load_hit_cnt <= diag_mem_load_hit_cnt + 32'd1;
        end
        if (diag_dcache_lookup_miss) begin
            case (u_dcache.choose_way)
                2'd0: diag_dcache_way0_victim_cnt <= diag_dcache_way0_victim_cnt + 32'd1;
                2'd1: diag_dcache_way1_victim_cnt <= diag_dcache_way1_victim_cnt + 32'd1;
                2'd2: diag_dcache_way2_victim_cnt <= diag_dcache_way2_victim_cnt + 32'd1;
                default: diag_dcache_way3_victim_cnt <= diag_dcache_way3_victim_cnt + 32'd1;
            endcase
            if (diag_dcache_req_is_store)
                diag_mem_store_miss_cnt <= diag_mem_store_miss_cnt + 32'd1;
            else
                diag_mem_load_miss_cnt <= diag_mem_load_miss_cnt + 32'd1;
        end

        // Attribute each blocked MEM-stage cycle to its PC and current cache
        // phase.  This identifies the dynamic load/store PCs that dominate time.
        if (diag_mem_waiting) begin
            diag_mem_stage_wait_cnt <= diag_mem_stage_wait_cnt + 32'd1;
            if (diag_mem_wait_uncache)
                diag_mem_uncache_stage_wait_cnt <= diag_mem_uncache_stage_wait_cnt + 32'd1;
            else if (diag_dcache_lookup)
                diag_mem_lookup_wait_cnt <= diag_mem_lookup_wait_cnt + 32'd1;
            else if (diag_dcache_wb_state)
                diag_mem_wb_wait_cnt <= diag_mem_wb_wait_cnt + 32'd1;
            else if (diag_dcache_rdreq_state)
                diag_mem_rdreq_wait_cnt <= diag_mem_rdreq_wait_cnt + 32'd1;
            else if (diag_dcache_refill_state)
                diag_mem_refill_wait_cnt <= diag_mem_refill_wait_cnt + 32'd1;
            else
                diag_mem_other_wait_cnt <= diag_mem_other_wait_cnt + 32'd1;

            diag_memory_hit_idx = -1;
            diag_memory_min_idx = 0;
            diag_memory_min_count = 32'hffff_ffff;
            for (diag_i = 0; diag_i < DIAG_MEMORY_HOT_SLOTS; diag_i = diag_i + 1) begin
                if (diag_memory_hot_valid[diag_i] &&
                    (diag_memory_hot_pc[diag_i] == diag_mem_wait_pc))
                    diag_memory_hit_idx = diag_i;
                if (!diag_memory_hot_valid[diag_i]) begin
                    diag_memory_min_idx = diag_i;
                    diag_memory_min_count = 32'b0;
                end else if (diag_memory_hot_wait[diag_i] < diag_memory_min_count) begin
                    diag_memory_min_idx = diag_i;
                    diag_memory_min_count = diag_memory_hot_wait[diag_i];
                end
            end
            if (diag_memory_hit_idx >= 0) begin
                diag_memory_hot_wait[diag_memory_hit_idx] <= diag_memory_hot_wait[diag_memory_hit_idx] + 32'd1;
                if (diag_mem_wait_load)
                    diag_memory_hot_load[diag_memory_hit_idx] <= diag_memory_hot_load[diag_memory_hit_idx] + 32'd1;
                else
                    diag_memory_hot_store[diag_memory_hit_idx] <= diag_memory_hot_store[diag_memory_hit_idx] + 32'd1;
                if (diag_dcache_lookup)
                    diag_memory_hot_lookup[diag_memory_hit_idx] <= diag_memory_hot_lookup[diag_memory_hit_idx] + 32'd1;
                if (diag_dcache_wb_state)
                    diag_memory_hot_wb[diag_memory_hit_idx] <= diag_memory_hot_wb[diag_memory_hit_idx] + 32'd1;
                if (diag_dcache_rdreq_state)
                    diag_memory_hot_rdreq[diag_memory_hit_idx] <= diag_memory_hot_rdreq[diag_memory_hit_idx] + 32'd1;
                if (diag_dcache_refill_state)
                    diag_memory_hot_refill[diag_memory_hit_idx] <= diag_memory_hot_refill[diag_memory_hit_idx] + 32'd1;
                if (diag_mem_wait_uncache)
                    diag_memory_hot_uncache[diag_memory_hit_idx] <= diag_memory_hot_uncache[diag_memory_hit_idx] + 32'd1;
            end else begin
                diag_memory_hot_valid[diag_memory_min_idx]   <= 1'b1;
                diag_memory_hot_pc[diag_memory_min_idx]      <= diag_mem_wait_pc;
                diag_memory_hot_wait[diag_memory_min_idx]    <= diag_memory_min_count + 32'd1;
                diag_memory_hot_load[diag_memory_min_idx]    <= diag_mem_wait_load ? 32'd1 : 32'd0;
                diag_memory_hot_store[diag_memory_min_idx]   <= diag_mem_wait_load ? 32'd0 : 32'd1;
                diag_memory_hot_lookup[diag_memory_min_idx]  <= diag_dcache_lookup ? 32'd1 : 32'd0;
                diag_memory_hot_wb[diag_memory_min_idx]      <= diag_dcache_wb_state ? 32'd1 : 32'd0;
                diag_memory_hot_rdreq[diag_memory_min_idx]   <= diag_dcache_rdreq_state ? 32'd1 : 32'd0;
                diag_memory_hot_refill[diag_memory_min_idx]  <= diag_dcache_refill_state ? 32'd1 : 32'd0;
                diag_memory_hot_uncache[diag_memory_min_idx] <= diag_mem_wait_uncache ? 32'd1 : 32'd0;
            end
        end
    end
end

// perf_count_stop_pulse is asserted one cycle after the second timer read.  The
// #1 delay lets nonblocking assignments settle before the console snapshot.
`ifndef ENABLE_M171W_WIDE_PROFILER
always @(posedge clk) begin
    if (perf_count_stop_pulse) begin
        #1;
        $display("PERF_DIAG_BEGIN|version=M17.5D-D12");
`ifdef DISABLE_M174I_4WAY_ICACHE
        $display("PERF_DIAG|icache_config|icache_kb=8|ways=2|sets=256|line_bytes=16|pipelined_hit=1");
`else
        $display("PERF_DIAG|icache_config|icache_kb=16|ways=4|sets=256|line_bytes=16|pipelined_hit=1");
`endif
`ifdef DISABLE_M170C_4WAY_DCACHE
        $display("PERF_DIAG|cache_config|dcache_kb=8|ways=2|sets=256|line_bytes=16|blocking=1|m175d=0");
`else
`ifdef DISABLE_M175D_32KB_DCACHE
        $display("PERF_DIAG|cache_config|dcache_kb=16|ways=4|sets=256|line_bytes=16|blocking=1|m175d=0");
`else
        $display("PERF_DIAG|cache_config|dcache_kb=32|ways=4|sets=512|line_bytes=16|blocking=1|m175d=1");
`endif
`endif
`ifdef DISABLE_M16_LANE_SWAP
`ifdef DISABLE_M162_ALU_MUL_SWAP
        $display("PERF_DIAG|timing_config|normal_dual_issue=1|reverse_mem_swap=0|reverse_mul_swap=0|critical_cone_pruned=1");
`else
        $display("PERF_DIAG|timing_config|normal_dual_issue=1|reverse_mem_swap=0|reverse_mul_swap=1|critical_cone_pruned=0");
`endif
`else
`ifdef DISABLE_M162_ALU_MUL_SWAP
        $display("PERF_DIAG|timing_config|normal_dual_issue=1|reverse_mem_swap=1|reverse_mul_swap=0|critical_cone_pruned=0");
`else
        $display("PERF_DIAG|timing_config|normal_dual_issue=1|reverse_mem_swap=1|reverse_mul_swap=1|critical_cone_pruned=0");
`endif
`endif
        $display("PERF_DIAG|summary|cycles=%0d|retired=%0d|rf_writes=%0d|ipc_x1000=%0d|dual_issue=%0d|slot1_retired=%0d",
                 u_mycpu_core.perf_core_cycle_cnt,
                 u_mycpu_core.perf_retire_cnt,
                 u_mycpu_core.perf_rf_write_cnt,
                 diag_ratio_x1000(u_mycpu_core.perf_retire_cnt,
                                  u_mycpu_core.perf_core_cycle_cnt),
                 u_mycpu_core.perf_dual_issue_cnt,
                 u_mycpu_core.perf_slot1_retire_cnt);
        $display("PERF_DIAG|branch|control=%0d|conditional=%0d|taken=%0d|mispredict=%0d|accuracy_x10000=%0d|direction_miss=%0d|target_miss=%0d|conditional_miss=%0d|second_slot_control=%0d|second_slot_taken=%0d|second_slot_miss=%0d|second_slot_accuracy_x10000=%0d",
                 u_mycpu_core.perf_ctrl_cnt,
                 u_mycpu_core.perf_cond_branch_cnt,
                 u_mycpu_core.perf_taken_cnt,
                 u_mycpu_core.perf_mispredict_cnt,
                 64'd10000 - diag_ratio_x10000(u_mycpu_core.perf_mispredict_cnt,
                                               u_mycpu_core.perf_ctrl_cnt),
                 diag_bp_direction_miss_cnt,
                 diag_bp_target_miss_cnt,
                 diag_bp_cond_miss_cnt,
                 diag_bp_second_slot_ctrl_cnt,
                 diag_bp_second_slot_taken_cnt,
                 diag_bp_second_slot_miss_cnt,
                 64'd10000 - diag_ratio_x10000(diag_bp_second_slot_miss_cnt,
                                               diag_bp_second_slot_ctrl_cnt));
        $display("PERF_DIAG|fetch|single_requests=%0d|dual_requests=%0d|dual_request_share_x10000=%0d|predicted_taken_requests=%0d|icache_misses=%0d|icache_refill_beats=%0d|if_wait_addr_cycles=%0d|if_wait_data_cycles=%0d|pending_cycles=%0d",
                 diag_fetch_single_req_cnt,
                 diag_fetch_dual_req_cnt,
                 diag_ratio_x10000(diag_fetch_dual_req_cnt,
                    diag_fetch_single_req_cnt + diag_fetch_dual_req_cnt),
                 diag_fetch_pred_taken_cnt,
                 perf_icache_miss_cnt,
                 perf_icache_refill_beat_cnt,
                 perf_if_wait_addr_cnt,
                 perf_if_wait_data_cnt,
                 diag_fetch_pending_cycle_cnt);
        $display("PERF_DIAG|ifq|empty_cycles=%0d|one_cycles=%0d|two_cycles=%0d|three_cycles=%0d|full_cycles=%0d|head_unready_cycles=%0d|frontend_starve_cycles=%0d|redirects=%0d|discarded_entries=%0d",
                 diag_ifq_empty_cycle_cnt,
                 diag_ifq_one_cycle_cnt,
                 diag_ifq_two_cycle_cnt,
                 diag_ifq_three_cycle_cnt,
                 diag_ifq_full_cycle_cnt,
                 diag_ifq_head_unready_cycle_cnt,
                 diag_frontend_starve_cycle_cnt,
                 diag_redirect_cnt,
                 diag_redirect_discard_entry_cnt);
        $display("PERF_DIAG|issue|main_issues=%0d|pair_candidates=%0d|dual_issues=%0d|pair_success_x10000=%0d|normal_pairs=%0d|swapped_pairs=%0d|fail_no_next=%0d|fail_boundary=%0d|fail_class=%0d|fail_dependency=%0d|fail_addrmode=%0d|fail_other=%0d|backend_block_cycles=%0d",
                 diag_main_issue_cnt,
                 diag_pair_candidate_cnt,
                 u_mycpu_core.perf_dual_issue_cnt,
                 diag_ratio_x10000(u_mycpu_core.perf_dual_issue_cnt,
                                   diag_pair_candidate_cnt),
                 diag_pair_normal_cnt,
                 diag_pair_swap_cnt,
                 diag_pair_fail_no_next_cnt,
                 diag_pair_fail_boundary_cnt,
                 diag_pair_fail_class_cnt,
                 diag_pair_fail_dependency_cnt,
                 diag_pair_fail_addrmode_cnt,
                 diag_pair_fail_other_cnt,
                 diag_backend_block_cycle_cnt);
        $display("PERF_DIAG|class_detail|mem=%0d|branch=%0d|mul=%0d|div=%0d|system=%0d|slot0=%0d|other=%0d",
                 diag_pair_fail_class_mem_cnt,
                 diag_pair_fail_class_branch_cnt,
                 diag_pair_fail_class_mul_cnt,
                 diag_pair_fail_class_div_cnt,
                 diag_pair_fail_class_system_cnt,
                 diag_pair_fail_class_slot0_cnt,
                 diag_pair_fail_class_other_cnt);
        $display("PERF_DIAG|swap_detail|memory=%0d|multiply=%0d",
                 diag_pair_swap_mem_cnt,
                 diag_pair_swap_mul_cnt);
        $display("PERF_DIAG|dependency_detail|pair_raw=%0d|pair_waw=%0d|main_exe=%0d|side_exe=%0d|mem_load=%0d",
                 diag_dep_pair_raw_cnt,
                 diag_dep_pair_waw_cnt,
                 diag_dep_main_exe_cnt,
                 diag_dep_side_exe_cnt,
                 diag_dep_mem_load_cnt);
        $display("PERF_DIAG|stall|id_blocked=%0d|exe_load=%0d|mem_load=%0d|branch_source=%0d|exe_raw=%0d|mem_address=%0d|system=%0d|tlb_precheck=%0d",
                 u_mycpu_core.perf_ds_blocked_cnt,
                 u_mycpu_core.perf_load_stall_cnt,
                 u_mycpu_core.perf_ms_load_stall_cnt,
                 u_mycpu_core.perf_branch_src_stall_cnt,
                 u_mycpu_core.perf_es_raw_stall_cnt,
                 u_mycpu_core.perf_mem_addr_es_stall_cnt,
                 u_mycpu_core.perf_sys_stall_cnt,
                 u_mycpu_core.perf_tlb_precheck_stall_cnt);
        $display("PERF_DIAG|memory|dcache_requests=%0d|dcache_read_misses=%0d|dcache_writebacks=%0d|dcache_refill_beats=%0d|uncache_requests=%0d|uncache_wait_cycles=%0d|axi_ar=%0d|axi_r_beats=%0d|axi_aw=%0d|axi_w_beats=%0d|axi_b=%0d",
                 perf_dcache_req_cnt,
                 perf_dcache_rd_miss_cnt,
                 perf_dcache_wb_cnt,
                 perf_dcache_refill_beat_cnt,
                 perf_uncache_req_cnt,
                 perf_uncache_wait_cnt,
                 perf_axi_ar_cnt,
                 perf_axi_r_beat_cnt,
                 perf_axi_aw_cnt,
                 perf_axi_w_beat_cnt,
                 perf_axi_b_cnt);
        $display("PERF_DIAG|memory_detail|load_req=%0d|store_req=%0d|load_hit=%0d|store_hit=%0d|load_miss=%0d|store_miss=%0d|mem_wait=%0d|lookup_wait=%0d|wb_wait=%0d|rdreq_wait=%0d|refill_wait=%0d|uncache_wait=%0d|other_wait=%0d",
                 diag_mem_load_req_cnt,
                 diag_mem_store_req_cnt,
                 diag_mem_load_hit_cnt,
                 diag_mem_store_hit_cnt,
                 diag_mem_load_miss_cnt,
                 diag_mem_store_miss_cnt,
                 diag_mem_stage_wait_cnt,
                 diag_mem_lookup_wait_cnt,
                 diag_mem_wb_wait_cnt,
                 diag_mem_rdreq_wait_cnt,
                 diag_mem_refill_wait_cnt,
                 diag_mem_uncache_stage_wait_cnt,
                 diag_mem_other_wait_cnt);
        $display("PERF_DIAG|cache_state|lookup_cycles=%0d|wb_cycles=%0d|rdreq_cycles=%0d|refill_cycles=%0d|refill_no_data=%0d|wb_not_ready=%0d|rd_not_ready=%0d",
                 diag_dcache_lookup_cycle_cnt,
                 diag_dcache_wb_state_cycle_cnt,
                 diag_dcache_rdreq_state_cycle_cnt,
                 diag_dcache_refill_state_cycle_cnt,
                 diag_dcache_refill_no_data_cycle_cnt,
                 diag_dcache_wb_not_ready_cycle_cnt,
                 diag_dcache_rd_not_ready_cycle_cnt);
        $display("PERF_DIAG|cache_way|hit0=%0d|hit1=%0d|hit2=%0d|hit3=%0d|victim0=%0d|victim1=%0d|victim2=%0d|victim3=%0d",
                 diag_dcache_way0_hit_cnt,
                 diag_dcache_way1_hit_cnt,
                 diag_dcache_way2_hit_cnt,
                 diag_dcache_way3_hit_cnt,
                 diag_dcache_way0_victim_cnt,
                 diag_dcache_way1_victim_cnt,
                 diag_dcache_way2_victim_cnt,
                 diag_dcache_way3_victim_cnt);
        for (diag_i = 0; diag_i < DIAG_BRANCH_HOT_SLOTS; diag_i = diag_i + 1) begin
            if (diag_branch_hot_valid[diag_i])
                $display("PERF_DIAG|branch_hot%0d|approx=1|pc=0x%08x|exec=%0d|miss=%0d|taken=%0d|conditional=%0d|backward=%0d|flips=%0d|accuracy_x10000=%0d",
                         diag_i,
                         diag_branch_hot_pc[diag_i],
                         diag_branch_hot_exec[diag_i],
                         diag_branch_hot_miss[diag_i],
                         diag_branch_hot_taken[diag_i],
                         diag_branch_hot_cond[diag_i],
                         diag_branch_hot_back[diag_i],
                         diag_branch_hot_flip[diag_i],
                         64'd10000 - diag_ratio_x10000(diag_branch_hot_miss[diag_i],
                                                       diag_branch_hot_exec[diag_i]));
        end
        for (diag_i = 0; diag_i < DIAG_MEMORY_HOT_SLOTS; diag_i = diag_i + 1) begin
            if (diag_memory_hot_valid[diag_i])
                $display("PERF_DIAG|memory_hot%0d|approx=1|pc=0x%08x|wait=%0d|load=%0d|store=%0d|lookup=%0d|wb=%0d|rdreq=%0d|refill=%0d|uncache=%0d",
                         diag_i,
                         diag_memory_hot_pc[diag_i],
                         diag_memory_hot_wait[diag_i],
                         diag_memory_hot_load[diag_i],
                         diag_memory_hot_store[diag_i],
                         diag_memory_hot_lookup[diag_i],
                         diag_memory_hot_wb[diag_i],
                         diag_memory_hot_rdreq[diag_i],
                         diag_memory_hot_refill[diag_i],
                         diag_memory_hot_uncache[diag_i]);
        end
        $display("PERF_DIAG_END|version=M17.5D-D12");
    end
end
`endif

`ifdef ENABLE_M171W_WIDE_PROFILER
`include "m17_1w_profiler.vh"
`endif
`endif

endmodule


//------------------------------------------------------------------------------
// 2x1 SRAM-like to AXI bridge for exp15.
//
// Design choices:
//   * single-beat AXI transfers only: AxLEN=0, WLAST=1;
//   * one outstanding read and one outstanding write at most;
//   * data read has priority over instruction fetch on the shared AR channel;
//   * SRAM-like addr_ok means the bridge has accepted and buffered the request;
//   * SRAM-like data_ok is generated by AXI RVALID for reads and BVALID for writes.
//------------------------------------------------------------------------------
module sram_axi_2x1_bridge(
    input  wire        clk,
    input  wire        resetn,

    // ICache memory-side interface
    input  wire        icache_rd_req,
    input  wire [ 2:0] icache_rd_type,
    input  wire [31:0] icache_rd_addr,
    output wire        icache_rd_rdy,
    output wire        icache_ret_valid,
    output wire        icache_ret_last,
    output wire [31:0] icache_ret_data,

    input  wire        icache_wr_req,
    input  wire [ 2:0] icache_wr_type,
    input  wire [31:0] icache_wr_addr,
    input  wire [ 3:0] icache_wr_wstrb,
    input  wire [127:0] icache_wr_data,
    output wire        icache_wr_rdy,

    // DCache memory-side interface
    input  wire        dcache_rd_req,
    input  wire [ 2:0] dcache_rd_type,
    input  wire [31:0] dcache_rd_addr,
    output wire        dcache_rd_rdy,
    output wire        dcache_ret_valid,
    output wire        dcache_ret_last,
    output wire [31:0] dcache_ret_data,

    input  wire        dcache_wr_req,
    input  wire [ 2:0] dcache_wr_type,
    input  wire [31:0] dcache_wr_addr,
    input  wire [ 3:0] dcache_wr_wstrb,
    input  wire [127:0] dcache_wr_data,
    output wire        dcache_wr_rdy,

    // Uncached data SRAM-like interface, used for confreg/MMIO space.
    input  wire        uncache_req,
    input  wire        uncache_wr,
    input  wire [ 1:0] uncache_size,
    input  wire [ 3:0] uncache_wstrb,
    input  wire [31:0] uncache_addr,
    input  wire [31:0] uncache_wdata,
    output wire        uncache_addr_ok,
    output wire        uncache_data_ok,
    output wire [31:0] uncache_rdata,

    // AXI read address channel
    output wire [ 3:0] arid,
    output wire [31:0] araddr,
    output wire [ 7:0] arlen,
    output wire [ 2:0] arsize,
    output wire [ 1:0] arburst,
    output wire [ 1:0] arlock,
    output wire [ 3:0] arcache,
    output wire [ 2:0] arprot,
    output wire        arvalid,
    input  wire        arready,

    // AXI read data channel
    input  wire [ 3:0] rid,
    input  wire [31:0] rdata,
    input  wire [ 1:0] rresp,
    input  wire        rlast,
    input  wire        rvalid,
    output wire        rready,

    // AXI write address channel
    output wire [ 3:0] awid,
    output wire [31:0] awaddr,
    output wire [ 7:0] awlen,
    output wire [ 2:0] awsize,
    output wire [ 1:0] awburst,
    output wire [ 1:0] awlock,
    output wire [ 3:0] awcache,
    output wire [ 2:0] awprot,
    output wire        awvalid,
    input  wire        awready,

    // AXI write data channel
    output wire [ 3:0] wid,
    output wire [31:0] wdata,
    output wire [ 3:0] wstrb,
    output wire        wlast,
    output wire        wvalid,
    input  wire        wready,

    // AXI write response channel
    input  wire [ 3:0] bid,
    input  wire [ 1:0] bresp,
    input  wire        bvalid,
    output wire        bready
);

//------------------------------------------------------------------------------
// exp22 ICache + DCache + uncached-data -> AXI bridge
//
// One transaction is issued at a time.  This is conservative but stable for the
// lab SoC and keeps the I/D cache integration local to this bridge.
// Priority when idle:
//   uncached data access > DCache writeback > DCache line refill > ICache line refill
//------------------------------------------------------------------------------

function [31:0] axi_addr_map;
    input [31:0] addr;
    begin
        if (addr[31:16] == 16'hbfaf || addr[31:16] == 16'h1faf) begin
            axi_addr_map = addr;
        end
        else if (addr[31:30] == 2'b10) begin
            axi_addr_map = {2'b00, addr[29:0]};
        end
        else begin
            axi_addr_map = addr;
        end
    end
endfunction

localparam S_IDLE = 3'd0;
localparam S_AR   = 3'd1;
localparam S_R    = 3'd2;
localparam S_AW   = 3'd3;
localparam S_W    = 3'd4;
localparam S_B    = 3'd5;

localparam TYPE_BYTE = 3'b000;
localparam TYPE_HALF = 3'b001;
localparam TYPE_WORD = 3'b010;
localparam TYPE_LINE = 3'b100;
localparam TYPE_FOUR_LINE = 3'b101;
localparam TYPE_TRIPLE_LINE = 3'b110;
localparam TYPE_FIVE_LINE = 3'b111;

localparam SRC_NONE    = 2'd0;
localparam SRC_ICACHE  = 2'd1;
localparam SRC_DCACHE  = 2'd2;
localparam SRC_UNCACHE = 2'd3;

reg [2:0]  state;
reg [1:0]  req_src;
reg        req_wr;
reg [1:0]  req_size;
reg [2:0]  req_type;
reg [31:0] req_addr;
reg [31:0] req_wdata_word;
reg [127:0] req_wdata_line;
reg [3:0]  req_wstrb;
reg [1:0]  wbeat_cnt;
reg        aw_done;
reg        w_done;
reg [31:0] rdata_buf;

wire idle;
wire start_uncache;
wire start_dcache_wr;
wire start_dcache_rd;
wire start_icache_rd;
wire start_req;
wire start_read;
wire start_write;

wire ar_hs;
wire r_hs;
wire aw_hs;
wire w_hs;
wire b_hs;
wire req_line;
wire [1:0] req_last_beat;
wire read_done;
wire write_done;
wire [31:0] read_data_now;
wire [31:0] line_wdata_beat;
wire        aw_done_nxt;
wire        w_done_nxt;
wire        w_last_hs;

assign idle = (state == S_IDLE);

assign start_uncache   = idle && uncache_req;
assign start_dcache_wr = idle && !uncache_req && dcache_wr_req;
assign start_dcache_rd = idle && !uncache_req && !dcache_wr_req && dcache_rd_req;
assign start_icache_rd = idle && !uncache_req && !dcache_wr_req && !dcache_rd_req && icache_rd_req;
assign start_req       = start_uncache | start_dcache_wr | start_dcache_rd | start_icache_rd;
assign start_write     = start_uncache ? uncache_wr : start_dcache_wr;
assign start_read      = start_req && !start_write;

assign ar_hs = arvalid && arready;
assign r_hs  = rvalid  && rready;
assign aw_hs = awvalid && awready;
assign w_hs  = wvalid  && wready;
assign b_hs  = bvalid  && bready;

wire req_triple_line = (req_type == TYPE_TRIPLE_LINE);
wire req_four_line = (req_type == TYPE_FOUR_LINE);
wire req_five_line = (req_type == TYPE_FIVE_LINE);
assign req_line      = (req_type == TYPE_LINE) || req_triple_line ||
                       req_four_line || req_five_line;
assign req_last_beat = req_line ? 2'd3 : 2'd0;
assign read_done     = (state == S_R) && r_hs && (!req_line || rlast);
assign write_done    = (state == S_B) && b_hs;
assign read_data_now = r_hs ? rdata : rdata_buf;
assign w_last_hs     = w_hs && (wbeat_cnt == req_last_beat);
assign aw_done_nxt   = aw_done || aw_hs;
assign w_done_nxt    = w_done  || w_last_hs;

always @(posedge clk) begin
    if (!resetn) begin
        state          <= S_IDLE;
        req_src        <= SRC_NONE;
        req_wr         <= 1'b0;
        req_size       <= 2'b00;
        req_type       <= TYPE_WORD;
        req_addr       <= 32'b0;
        req_wdata_word <= 32'b0;
        req_wdata_line <= 128'b0;
        req_wstrb      <= 4'b0;
        wbeat_cnt      <= 2'b0;
        aw_done        <= 1'b0;
        w_done         <= 1'b0;
        rdata_buf      <= 32'b0;
    end
    else begin
        if (r_hs) begin
            rdata_buf <= rdata;
        end

        case (state)
            S_IDLE: begin
                wbeat_cnt <= 2'b0;
                aw_done   <= 1'b0;
                w_done    <= 1'b0;
                if (start_req) begin
                    req_src  <= start_uncache   ? SRC_UNCACHE :
                                start_dcache_wr ? SRC_DCACHE  :
                                start_dcache_rd ? SRC_DCACHE  : SRC_ICACHE;
                    req_wr   <= start_write;
                    req_size <= start_uncache ? uncache_size : 2'b10;
                    req_type <= start_uncache ? ((uncache_size == 2'b00) ? TYPE_BYTE :
                                                 (uncache_size == 2'b01) ? TYPE_HALF : TYPE_WORD) :
                                start_dcache_wr ? dcache_wr_type :
                                start_dcache_rd ? dcache_rd_type : icache_rd_type;
                    req_addr       <= start_uncache   ? uncache_addr :
                                      start_dcache_wr ? dcache_wr_addr :
                                      start_dcache_rd ? dcache_rd_addr : icache_rd_addr;
                    req_wdata_word <= uncache_wdata;
                    req_wdata_line <= dcache_wr_data;
                    req_wstrb      <= start_uncache ? uncache_wstrb : 4'hf;
                    state          <= start_write ? S_AW : S_AR;
                end
            end

            S_AR: begin
                if (ar_hs) begin
                    state <= S_R;
                end
            end

            S_R: begin
                if (read_done) begin
                    state <= S_IDLE;
                end
            end

            // AXI write address and write data channels are independent.
            // Some lab AXI RAM configurations only make progress when AWVALID
            // and WVALID are presented together, so do not serialize AW before W.
            // Keep AWVALID until AW handshake; keep WVALID until all beats finish.
            S_AW: begin
                aw_done <= aw_done_nxt;
                w_done  <= w_done_nxt;

                if (w_hs && (wbeat_cnt != req_last_beat)) begin
                    wbeat_cnt <= wbeat_cnt + 2'b01;
                end

                if (aw_done_nxt && w_done_nxt) begin
                    state <= S_B;
                end
            end

            S_W: begin
                state <= S_AW;
            end

            S_B: begin
                if (b_hs) begin
                    state <= S_IDLE;
                end
            end

            default: begin
                state <= S_IDLE;
            end
        endcase
    end
end

assign line_wdata_beat = (wbeat_cnt == 2'd0) ? req_wdata_line[ 31:  0] :
                         (wbeat_cnt == 2'd1) ? req_wdata_line[ 63: 32] :
                         (wbeat_cnt == 2'd2) ? req_wdata_line[ 95: 64] :
                                               req_wdata_line[127: 96];

// Cache-side handshakes.
assign icache_rd_rdy    = start_icache_rd;
assign icache_ret_valid = (state == S_R) && r_hs && (req_src == SRC_ICACHE);
assign icache_ret_last  = icache_ret_valid && rlast;
assign icache_ret_data  = rdata;
assign icache_wr_rdy    = 1'b1;

assign dcache_rd_rdy    = start_dcache_rd;
assign dcache_ret_valid = (state == S_R) && r_hs && (req_src == SRC_DCACHE) && !req_wr;
assign dcache_ret_last  = dcache_ret_valid && rlast;
assign dcache_ret_data  = rdata;
assign dcache_wr_rdy    = start_dcache_wr;

// Uncached data responses keep exp19 behavior: addr_ok and data_ok are returned
// together at real AXI completion, so uncached MMIO access remains strongly ordered.
assign uncache_addr_ok = ((read_done || write_done) && (req_src == SRC_UNCACHE));
assign uncache_data_ok = ((read_done || write_done) && (req_src == SRC_UNCACHE));
assign uncache_rdata   = read_data_now;

// AXI read address channel.
assign arid     = (req_src == SRC_ICACHE) ? 4'd0 : 4'd1;
assign araddr   = axi_addr_map(req_addr);
assign arlen    = req_five_line   ? 8'd19 :
                  req_four_line   ? 8'd15 :
                  req_triple_line ? 8'd11 :
                  req_line        ? 8'd3 : 8'd0;
assign arsize   = req_line ? 3'b010 : {1'b0, req_size};
assign arburst  = 2'b01;
assign arlock   = 2'b00;
assign arcache  = 4'b0000;
assign arprot   = 3'b000;
assign arvalid  = (state == S_AR);
assign rready   = (state == S_R);

// AXI write address/data/response channels.
assign awid     = 4'd1;
assign awaddr   = axi_addr_map(req_addr);
assign awlen    = req_line ? 8'd3 : 8'd0;
assign awsize   = req_line ? 3'b010 : {1'b0, req_size};
assign awburst  = 2'b01;
assign awlock   = 2'b00;
assign awcache  = 4'b0000;
assign awprot   = 3'b000;
assign awvalid  = (state == S_AW) && !aw_done;

assign wid      = 4'd1;
assign wdata    = req_line ? line_wdata_beat : req_wdata_word;
assign wstrb    = req_line ? 4'hf : req_wstrb;
assign wlast    = (state == S_AW) && (wbeat_cnt == req_last_beat);
assign wvalid   = (state == S_AW) && !w_done;

assign bready   = (state == S_B);

wire unused_axi_resp;
wire unused_cache_wr;
assign unused_axi_resp = (|rid) | (|rresp) | (|bid) | (|bresp);
assign unused_cache_wr = icache_wr_req | (|icache_wr_type) | (|icache_wr_addr) |
                         (|icache_wr_wstrb) | (|icache_wr_data) | (|dcache_wr_wstrb);

endmodule

module mycpu_core(
    input         clk,
    input         resetn,
    input  [ 7:0] ext_int,
    input         perf_count_enable_i,
    input         perf_count_clear_i,
    // inst sram-like interface
    output        inst_sram_req,
    output        inst_sram_wr,
    output [ 1:0] inst_sram_size,
    output [ 3:0] inst_sram_wstrb,
    output [31:0] inst_sram_addr,
    output [31:0] inst_sram_wdata,
    output        inst_sram_dual,
    input         inst_sram_addr_ok,
    input         inst_sram_data_ok,
    input  [31:0] inst_sram_rdata,
    input  [31:0] inst_sram_rdata2,
    // data sram-like interface
    output        data_sram_req,
    output        data_sram_wr,
    output [ 1:0] data_sram_size,
    output [ 3:0] data_sram_wstrb,
    output [31:0] data_sram_addr,
    output [31:0] data_sram_addr_fast,
    output        data_sram_addr_late,
    output [31:0] data_sram_wdata,
    output        data_sram_uncached,
    input         data_sram_addr_ok,
    input         data_sram_data_ok,
    input  [31:0] data_sram_rdata,
    // CACOP sideband to cache modules
    output        cacop_req,
    output [ 4:0] cacop_code,
    output [31:0] cacop_addr,
    output [31:0] cacop_paddr,
    input         cacop_addr_ok,
    input         cacop_data_ok,
    // trace debug interface
    output [31:0] debug_wb_pc,
    output [ 3:0] debug_wb_rf_we,
    output [ 4:0] debug_wb_rf_wnum,
    output [31:0] debug_wb_rf_wdata
);
reg         reset;
always @(posedge clk) reset <= ~resetn;

wire         ds_allowin;
wire         es_allowin;
wire         ms_allowin;
wire         ws_allowin;
wire         fs_to_ds_valid;
wire         fs_to_ds_valid1;
wire [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus1;
wire [`SLOT1_META_WD -1:0] fs_slot1_meta;
wire         fs_pop_base;
wire         fs_pop_extra;

// Restricted dual-issue Slot-1 side lane. Slot 0 remains the original
// architectural pipeline; Slot 1 accepts only simple integer ALU operations.
wire         slot1_issue_valid;
wire [11:0]  slot1_issue_alu_op;
wire [31:0]  slot1_issue_src1;
wire [31:0]  slot1_issue_src2;
wire [ 4:0]  slot1_issue_dest;
wire [31:0]  slot1_issue_pc;
wire [31:0]  slot1_issue_inst;
wire         slot1_issue_is_older;
wire         slot1_issue_src1_from_main_ms;
wire         slot1_issue_src2_from_main_ms;
wire         slot1_issue_src1_from_main_ws;
wire         slot1_issue_src2_from_main_ws;
wire         slot1_issue_src1_from_side_ms;
wire         slot1_issue_src2_from_side_ms;
wire [ 4:0]  slot1_es_dest;
wire [ 4:0]  slot1_ms_dest;
wire [31:0]  slot1_ms_result;
wire         slot1_es_stage_valid;
wire         slot1_ms_stage_valid;
wire         slot1_ws_stage_valid;
wire         slot1_wb_we;
wire [ 4:0]  slot1_wb_waddr;
wire [31:0]  slot1_wb_wdata;
wire         slot1_wb_is_older;
wire [31:0]  slot1_wb_pc;
wire [31:0]  main_debug_wb_pc;
wire [ 3:0]  main_debug_wb_rf_we;
wire [ 4:0]  main_debug_wb_rf_wnum;
wire [31:0]  main_debug_wb_rf_wdata;

wire         ds_to_es_valid;
wire         es_to_ms_valid;
wire         ms_to_ws_valid;
wire [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus;
wire [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus;
wire [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus;
wire [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus;
wire [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus;
wire [`BR_BUS_WD       -1:0] br_bus;

// -------------------------------------------------------------------------
// Dynamic branch-prediction sideband.  The predictor is kept outside IF/ID
// so that it observes the fetch PC in IF and is trained only from the
// resolved architectural outcome in ID.
// -------------------------------------------------------------------------
wire [31:0] bp_query_pc;
wire        bp_pred_taken;
wire [31:0] bp_pred_nextpc;
wire        bp_pred_taken_slot1;
wire [31:0] bp_pred_nextpc_slot1;
wire        bp_update_en;
wire [31:0] bp_update_pc;
wire        bp_update_is_cond;
wire        bp_update_taken;
wire [31:0] bp_update_target;
wire        bp_update_is_call;
wire        bp_update_is_return;
wire        bp_update_is_indirect;

// ID resolves ordinary control transfers.  M17.8B adds a second, older
// resolution source for a conditional branch waiting behind a main-lane load.
// EXE has priority, and its resolve cycle blocks ID acceptance so the single
// predictor update port never receives two architectural branches at once.
wire        id_bp_update_en;
wire [31:0] id_bp_update_pc;
wire        id_bp_update_is_cond;
wire        id_bp_update_taken;
wire [31:0] id_bp_update_target;
wire        id_bp_update_is_call;
wire        id_bp_update_is_return;
wire        id_bp_update_is_indirect;
wire        es_deferred_bp_update_en;
wire [31:0] es_deferred_bp_update_pc;
wire        es_deferred_bp_update_taken;
wire [31:0] es_deferred_bp_update_target;
wire        es_deferred_bp_update_is_cond;
wire        es_deferred_bp_update_is_call;
wire        es_deferred_bp_update_is_return;
wire        es_deferred_bp_update_is_indirect;

// ID-stage stall classification for performance counters.
wire        perf_id_load_stall;
wire        perf_id_ms_load_stall;
wire        perf_id_branch_src_stall;
wire        perf_id_es_raw_stall;
wire        perf_id_mem_addr_es_stall;
wire        perf_id_sys_stall;
wire        perf_id_tlb_precheck_stall;
wire        perf_id_ds_blocked;
wire        perf_id_dual_issue;

wire        cacop_flush;
wire [31:0] cacop_flush_target;

// -------------------------------------------------------------------------
// Round 15 branch-recovery timing cut.
//
// Round 14 still registered a combined 32-bit redirect target selected by the
// current ID branch comparison.  The routed 100 MHz report showed the path
//   forwarded branch operand -> compare -> target mux -> id_br_target_r
// as the second timing limiter.  Register direction, taken target, fall-through
// and system target independently, then select only among registered values.
// This preserves the existing one-cycle recovery latency while removing the
// branch comparator from every target-register D input.
// -------------------------------------------------------------------------
wire        id_br_stall;
wire        id_normal_redirect_valid;
wire        id_normal_actual_taken;
wire [31:0] id_normal_taken_target;
wire [31:0] id_normal_fallthrough;
wire        id_sys_redirect_valid;
wire [31:0] id_sys_redirect_target;
wire        es_deferred_redirect_valid;
wire        es_deferred_actual_taken;
wire [31:0] es_deferred_taken_target;
wire [31:0] es_deferred_fallthrough;
wire        normal_redirect_valid_sel;
wire        normal_actual_taken_sel;
wire [31:0] normal_taken_target_sel;
wire [31:0] normal_fallthrough_sel;
wire        id_redirect_flush_r;
wire [31:0] id_redirect_target_r;
wire        id_external_flush;

// An EXE-resolved branch is older than the current ID instruction and therefore
// wins redirect arbitration.  The EXE stage prevents ID from being accepted on
// every deferred resolve cycle, so these fields never merge two commits.
assign normal_redirect_valid_sel = es_deferred_redirect_valid |
                                   id_normal_redirect_valid;
assign normal_actual_taken_sel = es_deferred_redirect_valid ?
                                 es_deferred_actual_taken :
                                 id_normal_actual_taken;
assign normal_taken_target_sel = es_deferred_redirect_valid ?
                                 es_deferred_taken_target :
                                 id_normal_taken_target;
assign normal_fallthrough_sel = es_deferred_redirect_valid ?
                                es_deferred_fallthrough :
                                id_normal_fallthrough;

`ifdef DISABLE_ROUND15_BRANCH_SPLIT
// A/B rollback: reconstruct the Round-14 combined target register.
wire        id_redirect_valid_comb = id_sys_redirect_valid | normal_redirect_valid_sel;
wire [31:0] id_redirect_target_comb = id_sys_redirect_valid ? id_sys_redirect_target :
                                      normal_actual_taken_sel ? normal_taken_target_sel :
                                                                normal_fallthrough_sel;
reg         id_br_taken_r;
reg  [31:0] id_br_target_r;
always @(posedge clk) begin
    if (reset || cacop_flush) begin
        id_br_taken_r  <= 1'b0;
        id_br_target_r <= 32'b0;
    end
    else begin
        id_br_taken_r  <= id_redirect_valid_comb;
        id_br_target_r <= id_redirect_target_comb;
    end
end
assign id_redirect_flush_r  = id_br_taken_r;
assign id_redirect_target_r = id_br_target_r;
`else
reg         id_normal_redirect_valid_r;
reg         id_normal_actual_taken_r;
reg  [31:0] id_normal_taken_target_r;
reg  [31:0] id_normal_fallthrough_r;
reg         id_sys_redirect_valid_r;
reg  [31:0] id_sys_redirect_target_r;

always @(posedge clk) begin
    if (reset || cacop_flush) begin
        id_normal_redirect_valid_r <= 1'b0;
        id_normal_actual_taken_r   <= 1'b0;
        id_normal_taken_target_r   <= 32'b0;
        id_normal_fallthrough_r    <= 32'b0;
        id_sys_redirect_valid_r    <= 1'b0;
        id_sys_redirect_target_r   <= 32'b0;
    end
    else begin
        id_normal_redirect_valid_r <= normal_redirect_valid_sel;
        id_normal_actual_taken_r   <= normal_actual_taken_sel;
        id_normal_taken_target_r   <= normal_taken_target_sel;
        id_normal_fallthrough_r    <= normal_fallthrough_sel;
        id_sys_redirect_valid_r    <= id_sys_redirect_valid;
        id_sys_redirect_target_r   <= id_sys_redirect_target;
    end
end

assign id_redirect_flush_r = id_sys_redirect_valid_r | id_normal_redirect_valid_r;
assign id_redirect_target_r = id_sys_redirect_valid_r ? id_sys_redirect_target_r :
                              id_normal_actual_taken_r ? id_normal_taken_target_r :
                                                         id_normal_fallthrough_r;
`endif

assign id_external_flush = cacop_flush | id_redirect_flush_r;
assign br_bus = cacop_flush ? {1'b0, 1'b1, cacop_flush_target} :
                               {id_br_stall, id_redirect_flush_r, id_redirect_target_r};

// RAW hazard information sent back to ID stage
wire [ 4:0] es_to_ds_dest;
wire [ 4:0] ms_to_ds_dest;
wire [ 4:0] ws_to_ds_dest;
wire        es_to_ds_load_op;
wire        es_to_ds_mem_load_op;
wire        ms_to_ds_load_op;
wire        ms_to_es_load_ready;
wire [31:0] ms_to_es_load_result;
// True when the current MEM-stage load will enter WB at this clock edge.
// ID can then release EXE-forwardable consumers; WB forwarding supplies the
// value in their first EXE cycle.
wire        ms_to_ds_load_leave;
// forwarding result buses sent back to ID stage
wire [31:0] es_to_ds_result;
wire [31:0] ms_to_ds_result;
wire [31:0] ws_to_ds_result;
wire        llbit_state;
wire        ll_w_commit;
wire        sc_w_commit;

// System/CSR instructions are serialized until older stages are empty.
wire        es_stage_valid;
wire        ms_stage_valid;
wire        ws_stage_valid;

// M17.8B predictor-update arbitration.  A deferred EXE branch is older than
// anything in ID.  EXE also withholds es_allowin on its resolve cycle, so the
// ID update should be zero; the explicit priority is retained as a safety net.
assign bp_update_en          = es_deferred_bp_update_en | id_bp_update_en;
assign bp_update_pc          = es_deferred_bp_update_en ? es_deferred_bp_update_pc :
                                                          id_bp_update_pc;
assign bp_update_is_cond     = es_deferred_bp_update_en ? es_deferred_bp_update_is_cond :
                                                          id_bp_update_is_cond;
assign bp_update_taken       = es_deferred_bp_update_en ? es_deferred_bp_update_taken :
                                                          id_bp_update_taken;
assign bp_update_target      = es_deferred_bp_update_en ? es_deferred_bp_update_target :
                                                          id_bp_update_target;
assign bp_update_is_call     = es_deferred_bp_update_en ? es_deferred_bp_update_is_call :
                                                          id_bp_update_is_call;
assign bp_update_is_return   = es_deferred_bp_update_en ? es_deferred_bp_update_is_return :
                                                          id_bp_update_is_return;
assign bp_update_is_indirect = es_deferred_bp_update_en ? es_deferred_bp_update_is_indirect :
                                                          id_bp_update_is_indirect;

// -------------------------------------------------------------------------
// 64-entry direct-mapped BTB + 2-bit BHT + 8-entry RAS.
// Prediction is queried with the virtual fetch PC.  The update interface is
// driven only when ID accepts a resolved control transfer into EXE.
// -------------------------------------------------------------------------
branch_predictor u_branch_predictor(
    .clk             (clk               ),
    // A CACOP instruction is the architectural synchronization point for
    // self-modifying code.  Invalidate BTB/BHT state with the I-cache flush so
    // a tagged PC-relative BTB target remains immutable between flushes.
    .reset           (reset | cacop_flush),
    .query_pc        (bp_query_pc       ),
    .pred_taken      (bp_pred_taken     ),
    .pred_nextpc     (bp_pred_nextpc    ),
    .pred_taken_slot1 (bp_pred_taken_slot1 ),
    .pred_nextpc_slot1(bp_pred_nextpc_slot1),
    .update_en       (bp_update_en      ),
    .update_pc       (bp_update_pc      ),
    .update_is_cond  (bp_update_is_cond ),
    .update_taken    (bp_update_taken   ),
    .update_target   (bp_update_target  ),
    .update_is_call    (bp_update_is_call    ),
    .update_is_return  (bp_update_is_return  ),
    .update_is_indirect(bp_update_is_indirect)
);

// IF stage
if_stage if_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ds_allowin     (ds_allowin     ),
    .fs_pop_base    (fs_pop_base    ),
    .fs_pop_extra   (fs_pop_extra   ),
    //brbus
    .br_bus         (br_bus         ),
    //outputs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    .fs_to_ds_valid1(fs_to_ds_valid1),
    .fs_to_ds_bus1  (fs_to_ds_bus1  ),
    .fs_slot1_meta  (fs_slot1_meta  ),
    // branch predictor query/result
    .bp_query_pc    (bp_query_pc    ),
    .bp_pred_taken  (bp_pred_taken  ),
    .bp_pred_nextpc (bp_pred_nextpc ),
    .bp_pred_taken_slot1  (bp_pred_taken_slot1  ),
    .bp_pred_nextpc_slot1 (bp_pred_nextpc_slot1 ),
    // inst sram interface
    .inst_sram_req    (inst_sram_req    ),
    .inst_sram_wr     (inst_sram_wr     ),
    .inst_sram_size   (inst_sram_size   ),
    .inst_sram_wstrb  (inst_sram_wstrb  ),
    .inst_sram_addr   (inst_sram_addr   ),
    .inst_sram_wdata  (inst_sram_wdata  ),
    .inst_sram_dual   (inst_sram_dual   ),
    .inst_sram_addr_ok(inst_sram_addr_ok),
    .inst_sram_data_ok(inst_sram_data_ok),
    .inst_sram_rdata  (inst_sram_rdata  ),
    .inst_sram_rdata2 (inst_sram_rdata2 )
);
// ID stage
id_stage id_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    .ext_int        (ext_int        ),
    //allowin
    .es_allowin     (es_allowin     ),
    .ds_allowin     (ds_allowin     ),
    //from fs
    .fs_to_ds_valid (fs_to_ds_valid ),
    .fs_to_ds_bus   (fs_to_ds_bus   ),
    .fs_to_ds_valid1(fs_to_ds_valid1),
    .fs_to_ds_bus1  (fs_to_ds_bus1  ),
    .fs_slot1_meta  (fs_slot1_meta  ),
    .fs_pop_base     (fs_pop_base     ),
    .fs_pop_extra    (fs_pop_extra    ),
    // restricted Slot-1 issue payload
    .slot1_issue_valid (slot1_issue_valid ),
    .slot1_issue_alu_op(slot1_issue_alu_op),
    .slot1_issue_src1  (slot1_issue_src1  ),
    .slot1_issue_src2  (slot1_issue_src2  ),
    .slot1_issue_dest  (slot1_issue_dest  ),
    .slot1_issue_pc    (slot1_issue_pc    ),
    .slot1_issue_inst  (slot1_issue_inst  ),
    .slot1_issue_is_older(slot1_issue_is_older),
    .slot1_issue_src1_from_main_ms(slot1_issue_src1_from_main_ms),
    .slot1_issue_src2_from_main_ms(slot1_issue_src2_from_main_ms),
    .slot1_issue_src1_from_main_ws(slot1_issue_src1_from_main_ws),
    .slot1_issue_src2_from_main_ws(slot1_issue_src2_from_main_ws),
    .slot1_issue_src1_from_side_ms(slot1_issue_src1_from_side_ms),
    .slot1_issue_src2_from_side_ms(slot1_issue_src2_from_side_ms),
    //to es
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to fs / registered recovery boundary
    .id_br_stall              (id_br_stall              ),
    .id_normal_redirect_valid (id_normal_redirect_valid ),
    .id_normal_actual_taken   (id_normal_actual_taken   ),
    .id_normal_taken_target   (id_normal_taken_target   ),
    .id_normal_fallthrough    (id_normal_fallthrough    ),
    .id_sys_redirect_valid    (id_sys_redirect_valid    ),
    .id_sys_redirect_target   (id_sys_redirect_target   ),
    .external_flush           (id_external_flush        ),
    // branch predictor resolved-update sideband
    .bp_update_en       (id_bp_update_en      ),
    .bp_update_pc       (id_bp_update_pc      ),
    .bp_update_is_cond  (id_bp_update_is_cond ),
    .bp_update_taken    (id_bp_update_taken   ),
    .bp_update_target   (id_bp_update_target  ),
    .bp_update_is_call    (id_bp_update_is_call    ),
    .bp_update_is_return  (id_bp_update_is_return  ),
    .bp_update_is_indirect(id_bp_update_is_indirect),
    .perf_load_stall_o        (perf_id_load_stall        ),
    .perf_ms_load_stall_o     (perf_id_ms_load_stall     ),
    .perf_branch_src_stall_o  (perf_id_branch_src_stall  ),
    .perf_es_raw_stall_o      (perf_id_es_raw_stall      ),
    .perf_mem_addr_es_stall_o (perf_id_mem_addr_es_stall ),
    .perf_sys_stall_o         (perf_id_sys_stall         ),
    .perf_tlb_precheck_stall_o(perf_id_tlb_precheck_stall),
    .perf_ds_blocked_o        (perf_id_ds_blocked        ),
    .perf_dual_issue_o        (perf_id_dual_issue        ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    //RAW hazard info from later stages
    .es_to_ds_dest  (es_to_ds_dest  ),
    .ms_to_ds_dest  (ms_to_ds_dest  ),
    .ws_to_ds_dest  (ws_to_ds_dest  ),
    .es_to_ds_load_op(es_to_ds_load_op),
    .es_to_ds_mem_load_op(es_to_ds_mem_load_op),
    .ms_to_ds_load_op(ms_to_ds_load_op),
    .ms_to_ds_load_leave(ms_to_ds_load_leave),
    .es_to_ds_result(es_to_ds_result),
    .ms_to_ds_result(ms_to_ds_result),
    .ws_to_ds_result(ws_to_ds_result),
    .es_stage_valid(es_stage_valid),
    .ms_stage_valid(ms_stage_valid),
    .ws_stage_valid(ws_stage_valid),
    // Slot-1 hazards, serialization and second RF write port
    .slot1_es_dest       (slot1_es_dest       ),
    .slot1_ms_dest       (slot1_ms_dest       ),
    .slot1_ms_result     (slot1_ms_result     ),
    .slot1_es_stage_valid(slot1_es_stage_valid),
    .slot1_ms_stage_valid(slot1_ms_stage_valid),
    .slot1_ws_stage_valid(slot1_ws_stage_valid),
    .slot1_wb_we         (slot1_wb_we         ),
    .slot1_wb_waddr      (slot1_wb_waddr      ),
    .slot1_wb_wdata      (slot1_wb_wdata      ),
    .ll_w_commit    (ll_w_commit    ),
    .sc_w_commit    (sc_w_commit    ),
    .llbit_state    (llbit_state    )
);

// Slot-1 follows the same EXE/MEM/WB backpressure as Slot 0. Slot 1 remains
// a one-cycle integer lane, while Slot 0 may execute any non-control, non-system
// instruction. Shared backpressure keeps both instructions aligned through
// variable-latency loads and multiply/divide operations.
dual_issue_lane u_dual_issue_lane(
    .clk                 (clk                 ),
    .reset               (reset               ),
    .external_flush      (cacop_flush         ),
    .main_es_allowin     (es_allowin          ),
    .main_ms_allowin     (ms_allowin          ),
    .main_ws_allowin     (ws_allowin          ),
    .main_es_to_ms_valid (es_to_ms_valid      ),
    .main_ms_to_ws_valid (ms_to_ws_valid      ),
    .issue_valid         (slot1_issue_valid   ),
    .issue_alu_op        (slot1_issue_alu_op  ),
    .issue_src1          (slot1_issue_src1    ),
    .issue_src2          (slot1_issue_src2    ),
    .issue_dest          (slot1_issue_dest    ),
    .issue_pc            (slot1_issue_pc      ),
    .issue_inst          (slot1_issue_inst    ),
    .issue_is_older      (slot1_issue_is_older),
    .issue_src1_from_main_ms(slot1_issue_src1_from_main_ms),
    .issue_src2_from_main_ms(slot1_issue_src2_from_main_ms),
    .issue_src1_from_main_ws(slot1_issue_src1_from_main_ws),
    .issue_src2_from_main_ws(slot1_issue_src2_from_main_ws),
    .issue_src1_from_side_ms(slot1_issue_src1_from_side_ms),
    .issue_src2_from_side_ms(slot1_issue_src2_from_side_ms),
    .main_ms_result      (ms_to_ds_result     ),
    .main_ws_result      (ws_to_ds_result     ),
    .es_dest             (slot1_es_dest       ),
    .ms_dest             (slot1_ms_dest       ),
    .ms_result           (slot1_ms_result     ),
    .es_stage_valid      (slot1_es_stage_valid),
    .ms_stage_valid      (slot1_ms_stage_valid),
    .ws_stage_valid      (slot1_ws_stage_valid),
    .wb_we               (slot1_wb_we         ),
    .wb_waddr            (slot1_wb_waddr      ),
    .wb_wdata            (slot1_wb_wdata      ),
    .wb_is_older       (slot1_wb_is_older   ),
    .wb_pc              (slot1_wb_pc          )
);

// EXE stage
exe_stage exe_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ms_allowin     (ms_allowin     ),
    .es_allowin     (es_allowin     ),
    .external_flush (cacop_flush    ),
    //from ds
    .ds_to_es_valid (ds_to_es_valid ),
    .ds_to_es_bus   (ds_to_es_bus   ),
    //to ms
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ds: RAW hazard info
    .es_to_ds_dest  (es_to_ds_dest  ),
    .es_to_ds_load_op(es_to_ds_load_op),
    .es_to_ds_mem_load_op(es_to_ds_mem_load_op),
    .es_to_ds_result(es_to_ds_result),
    .es_stage_valid(es_stage_valid),
    // M17.8B deferred load-dependent conditional branch resolution
    .deferred_bp_update_en    (es_deferred_bp_update_en    ),
    .deferred_bp_update_pc    (es_deferred_bp_update_pc    ),
    .deferred_bp_update_taken (es_deferred_bp_update_taken ),
    .deferred_bp_update_target(es_deferred_bp_update_target),
    .deferred_bp_update_is_cond(es_deferred_bp_update_is_cond),
    .deferred_bp_update_is_call(es_deferred_bp_update_is_call),
    .deferred_bp_update_is_return(es_deferred_bp_update_is_return),
    .deferred_bp_update_is_indirect(es_deferred_bp_update_is_indirect),
    .deferred_redirect_valid  (es_deferred_redirect_valid  ),
    .deferred_actual_taken    (es_deferred_actual_taken    ),
    .deferred_taken_target    (es_deferred_taken_target    ),
    .deferred_fallthrough     (es_deferred_fallthrough     ),
    // EXE-stage operand forwarding sources from MEM/WB
    .ms_to_ds_dest  (ms_to_ds_dest  ),
    .ms_to_ds_load_op(ms_to_ds_load_op),
    .ms_to_es_load_ready(ms_to_es_load_ready),
    .ms_to_es_load_result(ms_to_es_load_result),
    .ms_to_ds_result(ms_to_ds_result),
    .ws_to_ds_dest  (ws_to_ds_dest  ),
    .ws_to_ds_result(ws_to_ds_result),
    // data sram interface
    .data_sram_req    (data_sram_req    ),
    .data_sram_wr     (data_sram_wr     ),
    .data_sram_size   (data_sram_size   ),
    .data_sram_wstrb  (data_sram_wstrb  ),
    .data_sram_addr   (data_sram_addr   ),
    .data_sram_addr_fast(data_sram_addr_fast),
    .data_sram_addr_late(data_sram_addr_late),
    .data_sram_wdata  (data_sram_wdata  ),
    .data_sram_uncached(data_sram_uncached),
    .data_sram_addr_ok(data_sram_addr_ok),
    .cacop_req        (cacop_req        ),
    .cacop_code       (cacop_code       ),
    .cacop_addr       (cacop_addr       ),
    .cacop_paddr      (cacop_paddr      ),
    .cacop_addr_ok    (cacop_addr_ok    ),
    .llbit_state     (llbit_state     )
);
// MEM stage
mem_stage mem_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    .ms_allowin     (ms_allowin     ),
    //from es
    .es_to_ms_valid (es_to_ms_valid ),
    .es_to_ms_bus   (es_to_ms_bus   ),
    //to ws
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //to ds: RAW hazard info / forwarding result
    .ms_to_ds_dest  (ms_to_ds_dest  ),
    .ms_to_ds_load_op(ms_to_ds_load_op),
    .ms_to_ds_load_leave(ms_to_ds_load_leave),
    .ms_to_es_load_ready(ms_to_es_load_ready),
    .ms_to_es_load_result(ms_to_es_load_result),
    .ms_to_ds_result(ms_to_ds_result),
    .ms_stage_valid(ms_stage_valid),
    .cacop_flush      (cacop_flush      ),
    .cacop_flush_target(cacop_flush_target),
    //from data-sram
    .data_sram_data_ok(data_sram_data_ok | cacop_data_ok),
    .data_sram_rdata  (data_sram_rdata  )
);
// WB stage
wb_stage wb_stage(
    .clk            (clk            ),
    .reset          (reset          ),
    //allowin
    .ws_allowin     (ws_allowin     ),
    //from ms
    .ms_to_ws_valid (ms_to_ws_valid ),
    .ms_to_ws_bus   (ms_to_ws_bus   ),
    //to rf: for write back
    .ws_to_rf_bus   (ws_to_rf_bus   ),
    //to ds: RAW hazard info / forwarding result
    .ws_to_ds_dest  (ws_to_ds_dest  ),
    .ws_to_ds_result(ws_to_ds_result),
    .ws_stage_valid(ws_stage_valid),
    .ll_w_commit    (ll_w_commit    ),
    .sc_w_commit    (sc_w_commit    ),
    //trace debug interface
    .debug_wb_pc      (main_debug_wb_pc      ),
    .debug_wb_rf_we   (main_debug_wb_rf_we   ),
    .debug_wb_rf_wnum (main_debug_wb_rf_wnum ),
    .debug_wb_rf_wdata(main_debug_wb_rf_wdata)
);

// The external legacy debug port is single-wide.  Preserve program order by
// reporting the older side-lane instruction when a pair was physically swapped;
// normal pairs continue to report the older main-lane instruction.
assign debug_wb_pc       = slot1_wb_is_older ? slot1_wb_pc       : main_debug_wb_pc;
assign debug_wb_rf_we    = slot1_wb_is_older ? {4{slot1_wb_we}}  : main_debug_wb_rf_we;
assign debug_wb_rf_wnum  = slot1_wb_is_older ? slot1_wb_waddr    : main_debug_wb_rf_wnum;
assign debug_wb_rf_wdata = slot1_wb_is_older ? slot1_wb_wdata    : main_debug_wb_rf_wdata;

// -----------------------------------------------------------------------------
// Performance counters, core/pipeline side.
// The counters are non-architectural debug registers.  They are visible in
// Verilator/Vivado hierarchical waves under u_mycpu_core.perf_*.
// -----------------------------------------------------------------------------
`ifndef DISABLE_PERF_COUNTERS
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_core_cycle_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_retire_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_rf_write_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_ctrl_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_cond_branch_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_taken_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_mispredict_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_load_stall_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_ms_load_stall_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_branch_src_stall_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_es_raw_stall_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_mem_addr_es_stall_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_sys_stall_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_tlb_precheck_stall_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_ds_blocked_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_dual_issue_cnt;
(* keep = "true", mark_debug = "true" *) reg [31:0] perf_slot1_retire_cnt;

wire [1:0] perf_retire_inc = {1'b0, ws_stage_valid} +
                             {1'b0, slot1_ws_stage_valid};
wire [1:0] perf_rf_write_inc = {1'b0, (|main_debug_wb_rf_we)} +
                               {1'b0, slot1_wb_we};
wire perf_mispredict_fire = bp_update_en &&
                            (id_normal_redirect_valid | es_deferred_redirect_valid);

always @(posedge clk) begin
    if (reset || perf_count_clear_i) begin
        perf_core_cycle_cnt         <= 32'b0;
        perf_retire_cnt             <= 32'b0;
        perf_rf_write_cnt           <= 32'b0;
        perf_ctrl_cnt               <= 32'b0;
        perf_cond_branch_cnt        <= 32'b0;
        perf_taken_cnt              <= 32'b0;
        perf_mispredict_cnt         <= 32'b0;
        perf_load_stall_cnt         <= 32'b0;
        perf_ms_load_stall_cnt      <= 32'b0;
        perf_branch_src_stall_cnt   <= 32'b0;
        perf_es_raw_stall_cnt       <= 32'b0;
        perf_mem_addr_es_stall_cnt  <= 32'b0;
        perf_sys_stall_cnt          <= 32'b0;
        perf_tlb_precheck_stall_cnt <= 32'b0;
        perf_ds_blocked_cnt         <= 32'b0;
        perf_dual_issue_cnt         <= 32'b0;
        perf_slot1_retire_cnt       <= 32'b0;
    end else if (perf_count_enable_i) begin
        perf_core_cycle_cnt <= perf_core_cycle_cnt + 32'd1;

        if (perf_retire_inc != 2'd0)
            perf_retire_cnt <= perf_retire_cnt + {{30{1'b0}}, perf_retire_inc};
        if (perf_rf_write_inc != 2'd0)
            perf_rf_write_cnt <= perf_rf_write_cnt + {{30{1'b0}}, perf_rf_write_inc};
        if (perf_id_dual_issue)
            perf_dual_issue_cnt <= perf_dual_issue_cnt + 32'd1;
        if (slot1_ws_stage_valid)
            perf_slot1_retire_cnt <= perf_slot1_retire_cnt + 32'd1;

        if (bp_update_en)
            perf_ctrl_cnt <= perf_ctrl_cnt + 32'd1;
        if (bp_update_en && bp_update_is_cond)
            perf_cond_branch_cnt <= perf_cond_branch_cnt + 32'd1;
        if (bp_update_en && bp_update_taken)
            perf_taken_cnt <= perf_taken_cnt + 32'd1;
        if (perf_mispredict_fire)
            perf_mispredict_cnt <= perf_mispredict_cnt + 32'd1;

        if (perf_id_load_stall)
            perf_load_stall_cnt <= perf_load_stall_cnt + 32'd1;
        if (perf_id_ms_load_stall)
            perf_ms_load_stall_cnt <= perf_ms_load_stall_cnt + 32'd1;
        if (perf_id_branch_src_stall)
            perf_branch_src_stall_cnt <= perf_branch_src_stall_cnt + 32'd1;
        if (perf_id_es_raw_stall)
            perf_es_raw_stall_cnt <= perf_es_raw_stall_cnt + 32'd1;
        if (perf_id_mem_addr_es_stall)
            perf_mem_addr_es_stall_cnt <= perf_mem_addr_es_stall_cnt + 32'd1;
        if (perf_id_sys_stall)
            perf_sys_stall_cnt <= perf_sys_stall_cnt + 32'd1;
        if (perf_id_tlb_precheck_stall)
            perf_tlb_precheck_stall_cnt <= perf_tlb_precheck_stall_cnt + 32'd1;
        if (perf_id_ds_blocked)
            perf_ds_blocked_cnt <= perf_ds_blocked_cnt + 32'd1;
    end
end
`endif

endmodule



// Stage4 max-performance front-end: IF1 request + IF2 two-entry fetch queue.
// This changes the effective front-end pipeline from a single IF holding slot to
// a decoupled request/response front end.  The queue lets IF continue fetching
// one instruction ahead while ID is stalled, reducing front-end bubbles without
// changing the external SRAM-like interface or the IF->ID bus format.
module if_stage(
    input                          clk            ,
    input                          reset          ,
    input                          ds_allowin     ,
    input                          fs_pop_base    ,
    input                          fs_pop_extra   ,
    input  [`BR_BUS_WD       -1:0] br_bus         ,
    output                         fs_to_ds_valid ,
    output [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus   ,
    output                         fs_to_ds_valid1,
    output [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus1  ,
    output [`SLOT1_META_WD -1:0]   fs_slot1_meta  ,
    output [31:0]                  bp_query_pc    ,
    input                          bp_pred_taken  ,
    input  [31:0]                  bp_pred_nextpc ,
    input                          bp_pred_taken_slot1  ,
    input  [31:0]                  bp_pred_nextpc_slot1 ,
    output                         inst_sram_req    ,
    output                         inst_sram_wr     ,
    output [ 1:0]                  inst_sram_size   ,
    output [ 3:0]                  inst_sram_wstrb  ,
    output [31:0]                  inst_sram_addr   ,
    output [31:0]                  inst_sram_wdata  ,
    output                         inst_sram_dual   ,
    input                          inst_sram_addr_ok,
    input                          inst_sram_data_ok,
    input  [31:0]                  inst_sram_rdata  ,
    input  [31:0]                  inst_sram_rdata2
);

(* max_fanout = 8 *) wire        br_stall;
(* max_fanout = 8 *) wire        br_taken;
(* max_fanout = 8 *) wire [31:0] br_target;
assign {br_stall, br_taken, br_target} = br_bus;

// M16 functional dual-issue frontend.  A cache request may reserve one or two
// FIFO entries.  Dual requests are restricted to adjacent words in one 16-byte
// line and to a not-taken first-slot prediction; control-flow correctness is
// still recovered by the existing ID redirect machinery.
function [`SLOT1_META_WD-1:0] slot1_predecode;
    input [31:0] inst;
    reg [ 5:0] op_31_26_f;
    reg [ 3:0] op_25_22_f;
    reg [ 1:0] op_21_20_f;
    reg [ 4:0] op_19_15_f;
    reg        f_add_w, f_sub_w, f_slt, f_sltu;
    reg        f_nor, f_and, f_or, f_xor;
    reg        f_sll_w, f_srl_w, f_sra_w;
    reg        f_slli_w, f_srli_w, f_srai_w;
    reg        f_addi_w, f_slti, f_sltui;
    reg        f_andi, f_ori, f_xori;
    reg        f_lu12i_w, f_pcaddu12i;
    reg        f_ld_b, f_ld_h, f_ld_w, f_ld_bu, f_ld_hu;
    reg        f_st_b, f_st_h, f_st_w;
    reg        f_simple, f_mem, f_load, f_store;
    reg        f_src1_used, f_src2_used, f_src2_is_rd, f_src1_is_pc;
    reg [ 1:0] f_imm_sel;
    reg [11:0] f_alu_op;
    reg [ 7:0] f_mem_op;
    begin
        op_31_26_f = inst[31:26];
        op_25_22_f = inst[25:22];
        op_21_20_f = inst[21:20];
        op_19_15_f = inst[19:15];

        f_add_w  = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h00);
        f_sub_w  = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h02);
        f_slt    = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h04);
        f_sltu   = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h05);
        f_nor    = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h08);
        f_and    = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h09);
        f_or     = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h0a);
        f_xor    = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h0b);
        f_sll_w  = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h0e);
        f_srl_w  = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h0f);
        f_sra_w  = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h0) && (op_21_20_f == 2'h1) && (op_19_15_f == 5'h10);
        f_slli_w = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h1) && (op_21_20_f == 2'h0) && (op_19_15_f == 5'h01);
        f_srli_w = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h1) && (op_21_20_f == 2'h0) && (op_19_15_f == 5'h09);
        f_srai_w = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h1) && (op_21_20_f == 2'h0) && (op_19_15_f == 5'h11);
        f_addi_w = (op_31_26_f == 6'h00) && (op_25_22_f == 4'ha);
        f_slti   = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h8);
        f_sltui  = (op_31_26_f == 6'h00) && (op_25_22_f == 4'h9);
        f_andi   = (op_31_26_f == 6'h00) && (op_25_22_f == 4'hd);
        f_ori    = (op_31_26_f == 6'h00) && (op_25_22_f == 4'he);
        f_xori   = (op_31_26_f == 6'h00) && (op_25_22_f == 4'hf);
        f_lu12i_w    = (op_31_26_f == 6'h05) && !inst[25];
        f_pcaddu12i  = (op_31_26_f == 6'h07) && !inst[25];

        f_ld_b  = (op_31_26_f == 6'h0a) && (op_25_22_f == 4'h0);
        f_ld_h  = (op_31_26_f == 6'h0a) && (op_25_22_f == 4'h1);
        f_ld_w  = (op_31_26_f == 6'h0a) && (op_25_22_f == 4'h2);
        f_st_b  = (op_31_26_f == 6'h0a) && (op_25_22_f == 4'h4);
        f_st_h  = (op_31_26_f == 6'h0a) && (op_25_22_f == 4'h5);
        f_st_w  = (op_31_26_f == 6'h0a) && (op_25_22_f == 4'h6);
        f_ld_bu = (op_31_26_f == 6'h0a) && (op_25_22_f == 4'h8);
        f_ld_hu = (op_31_26_f == 6'h0a) && (op_25_22_f == 4'h9);

        f_simple = f_add_w | f_sub_w | f_slt | f_sltu |
                   f_nor | f_and | f_or | f_xor |
                   f_slli_w | f_srli_w | f_srai_w |
                   f_addi_w | f_slti | f_sltui |
                   f_andi | f_ori | f_xori |
                   f_sll_w | f_srl_w | f_sra_w |
                   f_lu12i_w | f_pcaddu12i;
        f_load  = f_ld_b | f_ld_h | f_ld_w | f_ld_bu | f_ld_hu;
        f_store = f_st_b | f_st_h | f_st_w;
        f_mem   = f_load | f_store;

        f_src1_used = (f_simple && !(f_lu12i_w | f_pcaddu12i)) | f_mem;
        f_src2_used = f_add_w | f_sub_w | f_slt | f_sltu |
                      f_nor | f_and | f_or | f_xor |
                      f_sll_w | f_srl_w | f_sra_w | f_store;
        f_src2_is_rd = f_store;
        f_src1_is_pc = f_pcaddu12i;

        f_imm_sel = (f_lu12i_w | f_pcaddu12i) ? 2'b11 :
                    (f_slli_w | f_srli_w | f_srai_w) ? 2'b10 :
                    (f_andi | f_ori | f_xori) ? 2'b01 : 2'b00;

        f_alu_op[ 0] = f_add_w | f_addi_w | f_pcaddu12i | f_mem;
        f_alu_op[ 1] = f_sub_w;
        f_alu_op[ 2] = f_slt | f_slti;
        f_alu_op[ 3] = f_sltu | f_sltui;
        f_alu_op[ 4] = f_and | f_andi;
        f_alu_op[ 5] = f_nor;
        f_alu_op[ 6] = f_or | f_ori;
        f_alu_op[ 7] = f_xor | f_xori;
        f_alu_op[ 8] = f_slli_w | f_sll_w;
        f_alu_op[ 9] = f_srli_w | f_srl_w;
        f_alu_op[10] = f_srai_w | f_sra_w;
        f_alu_op[11] = f_lu12i_w;

        f_mem_op = {f_st_w, f_st_h, f_st_b, f_ld_hu,
                    f_ld_bu, f_ld_w, f_ld_h, f_ld_b};

        slot1_predecode = {f_simple, f_mem, f_load, f_store,
                           f_src1_used, f_src2_used, f_src2_is_rd,
                           f_src1_is_pc, f_imm_sel, f_alu_op, f_mem_op};
    end
endfunction

function slot0_dual_predecode;
    input [31:0] inst;
    reg [5:0] op31;
    reg [3:0] op25;
    reg [1:0] op21;
    reg [4:0] op19;
    reg       is_reg_alu;
    reg       is_shift_imm;
    reg       is_alu_imm;
    reg       is_upper_imm;
    reg       is_memory;
    reg       is_muldiv;
    reg       is_llsc;
    begin
        op31 = inst[31:26];
        op25 = inst[25:22];
        op21 = inst[21:20];
        op19 = inst[19:15];

        // This bit is an exact whitelist, not merely a "not control/system"
        // classification.  Consequently an illegal or privileged instruction
        // can never enter the fast pair-admission cone in ID.
        is_reg_alu = (op31 == 6'h00) && (op25 == 4'h0) &&
                     (op21 == 2'h1) &&
                     ((op19 == 5'h00) || (op19 == 5'h02) ||
                      (op19 == 5'h04) || (op19 == 5'h05) ||
                      (op19 == 5'h08) || (op19 == 5'h09) ||
                      (op19 == 5'h0a) || (op19 == 5'h0b) ||
                      (op19 == 5'h0e) || (op19 == 5'h0f) ||
                      (op19 == 5'h10));
        is_shift_imm = (op31 == 6'h00) && (op25 == 4'h1) &&
                       (op21 == 2'h0) &&
                       ((op19 == 5'h01) || (op19 == 5'h09) ||
                        (op19 == 5'h11));
        is_alu_imm = (op31 == 6'h00) &&
                     ((op25 == 4'h8) || (op25 == 4'h9) ||
                      (op25 == 4'ha) || (op25 == 4'hd) ||
                      (op25 == 4'he) || (op25 == 4'hf));
        is_upper_imm = ((op31 == 6'h05) || (op31 == 6'h07)) &&
                       !inst[25];
        is_memory = (op31 == 6'h0a) &&
                    ((op25 == 4'h0) || (op25 == 4'h1) ||
                     (op25 == 4'h2) || (op25 == 4'h4) ||
                     (op25 == 4'h5) || (op25 == 4'h6) ||
                     (op25 == 4'h8) || (op25 == 4'h9));
        is_muldiv = (op31 == 6'h00) && (op25 == 4'h0) &&
                    (((op21 == 2'h1) &&
                      ((op19 == 5'h18) || (op19 == 5'h19) ||
                       (op19 == 5'h1a))) ||
                     ((op21 == 2'h2) && (op19 <= 5'h03)));
        is_llsc = (inst[31:24] == 8'h20) || (inst[31:24] == 8'h21);

        // M19.2 timing boundary: only exception-free integer instructions use
        // the zero-latency pair path.  Memory/LLSC pair eligibility previously
        // pulled effective-address alignment and data-TLB exception logic back
        // into IFQ pop control.  Those classes still execute normally in Slot 0.
        slot0_dual_predecode = is_reg_alu | is_shift_imm | is_alu_imm |
                               is_upper_imm;
    end
endfunction

wire [`SLOT1_META_WD-1:0] response_slot1_meta0 = slot1_predecode(inst_sram_rdata);
wire [`SLOT1_META_WD-1:0] response_slot1_meta1 = slot1_predecode(inst_sram_rdata2);
wire response_slot0_dual_ok0 = slot0_dual_predecode(inst_sram_rdata);
wire response_slot0_dual_ok1 = slot0_dual_predecode(inst_sram_rdata2);

reg [31:0] q0_pc, q0_inst, q0_pred_nextpc;
reg        q0_pred_taken, q0_slot0_dual_ok, q0_req_dual;
reg [`SLOT1_META_WD-1:0] q0_slot1_meta;
reg [31:0] q1_pc, q1_inst, q1_pred_nextpc;
reg        q1_pred_taken, q1_slot0_dual_ok, q1_req_dual;
reg [`SLOT1_META_WD-1:0] q1_slot1_meta;
reg [31:0] q2_pc, q2_inst, q2_pred_nextpc;
reg        q2_pred_taken, q2_slot0_dual_ok, q2_req_dual;
reg [`SLOT1_META_WD-1:0] q2_slot1_meta;
reg [31:0] q3_pc, q3_inst, q3_pred_nextpc;
reg        q3_pred_taken, q3_slot0_dual_ok, q3_req_dual;
reg [`SLOT1_META_WD-1:0] q3_slot1_meta;
reg [31:0] q4_pc, q4_inst, q4_pred_nextpc;
reg        q4_pred_taken, q4_slot0_dual_ok, q4_req_dual;
reg [`SLOT1_META_WD-1:0] q4_slot1_meta;
reg [31:0] q5_pc, q5_inst, q5_pred_nextpc;
reg        q5_pred_taken, q5_slot0_dual_ok, q5_req_dual;
reg [`SLOT1_META_WD-1:0] q5_slot1_meta;
reg [31:0] q6_pc, q6_inst, q6_pred_nextpc;
reg        q6_pred_taken, q6_slot0_dual_ok, q6_req_dual;
reg [`SLOT1_META_WD-1:0] q6_slot1_meta;
reg [31:0] q7_pc, q7_inst, q7_pred_nextpc;
reg        q7_pred_taken, q7_slot0_dual_ok, q7_req_dual;
reg [`SLOT1_META_WD-1:0] q7_slot1_meta;

reg [7:0] slot_ready;
reg [2:0] head_ptr, tail_ptr, resp_ptr;
reg [3:0] queue_count;
reg [3:0] live_pending_count;
reg       req_pending, req_dual;
reg [31:0] req_pc, req_pred_nextpc, req_pred_nextpc_slot1, nextpc_reg;
reg       req_pred_taken, req_pred_taken_slot1;
reg [3:0] discard_count;

reg [31:0] head_pc, head_inst, head_pred_nextpc;
reg        head_pred_taken, head_slot0_dual_ok;
reg [`SLOT1_META_WD-1:0] head_slot1_meta;
reg [31:0] head1_pc, head1_inst, head1_pred_nextpc;
reg        head1_pred_taken, head1_slot0_dual_ok;
wire [2:0] head1_ptr = head_ptr + 3'd1;

always @(*) begin
    case (head_ptr)
        3'd0: begin head_pc=q0_pc; head_inst=q0_inst; head_pred_taken=q0_pred_taken; head_pred_nextpc=q0_pred_nextpc; head_slot1_meta=q0_slot1_meta; head_slot0_dual_ok=q0_slot0_dual_ok; end
        3'd1: begin head_pc=q1_pc; head_inst=q1_inst; head_pred_taken=q1_pred_taken; head_pred_nextpc=q1_pred_nextpc; head_slot1_meta=q1_slot1_meta; head_slot0_dual_ok=q1_slot0_dual_ok; end
        3'd2: begin head_pc=q2_pc; head_inst=q2_inst; head_pred_taken=q2_pred_taken; head_pred_nextpc=q2_pred_nextpc; head_slot1_meta=q2_slot1_meta; head_slot0_dual_ok=q2_slot0_dual_ok; end
        3'd3: begin head_pc=q3_pc; head_inst=q3_inst; head_pred_taken=q3_pred_taken; head_pred_nextpc=q3_pred_nextpc; head_slot1_meta=q3_slot1_meta; head_slot0_dual_ok=q3_slot0_dual_ok; end
        3'd4: begin head_pc=q4_pc; head_inst=q4_inst; head_pred_taken=q4_pred_taken; head_pred_nextpc=q4_pred_nextpc; head_slot1_meta=q4_slot1_meta; head_slot0_dual_ok=q4_slot0_dual_ok; end
        3'd5: begin head_pc=q5_pc; head_inst=q5_inst; head_pred_taken=q5_pred_taken; head_pred_nextpc=q5_pred_nextpc; head_slot1_meta=q5_slot1_meta; head_slot0_dual_ok=q5_slot0_dual_ok; end
        3'd6: begin head_pc=q6_pc; head_inst=q6_inst; head_pred_taken=q6_pred_taken; head_pred_nextpc=q6_pred_nextpc; head_slot1_meta=q6_slot1_meta; head_slot0_dual_ok=q6_slot0_dual_ok; end
        default: begin head_pc=q7_pc; head_inst=q7_inst; head_pred_taken=q7_pred_taken; head_pred_nextpc=q7_pred_nextpc; head_slot1_meta=q7_slot1_meta; head_slot0_dual_ok=q7_slot0_dual_ok; end
    endcase
    case (head1_ptr)
        3'd0: begin head1_pc=q0_pc; head1_inst=q0_inst; head1_pred_taken=q0_pred_taken; head1_pred_nextpc=q0_pred_nextpc; head1_slot0_dual_ok=q0_slot0_dual_ok; end
        3'd1: begin head1_pc=q1_pc; head1_inst=q1_inst; head1_pred_taken=q1_pred_taken; head1_pred_nextpc=q1_pred_nextpc; head1_slot0_dual_ok=q1_slot0_dual_ok; end
        3'd2: begin head1_pc=q2_pc; head1_inst=q2_inst; head1_pred_taken=q2_pred_taken; head1_pred_nextpc=q2_pred_nextpc; head1_slot0_dual_ok=q2_slot0_dual_ok; end
        3'd3: begin head1_pc=q3_pc; head1_inst=q3_inst; head1_pred_taken=q3_pred_taken; head1_pred_nextpc=q3_pred_nextpc; head1_slot0_dual_ok=q3_slot0_dual_ok; end
        3'd4: begin head1_pc=q4_pc; head1_inst=q4_inst; head1_pred_taken=q4_pred_taken; head1_pred_nextpc=q4_pred_nextpc; head1_slot0_dual_ok=q4_slot0_dual_ok; end
        3'd5: begin head1_pc=q5_pc; head1_inst=q5_inst; head1_pred_taken=q5_pred_taken; head1_pred_nextpc=q5_pred_nextpc; head1_slot0_dual_ok=q5_slot0_dual_ok; end
        3'd6: begin head1_pc=q6_pc; head1_inst=q6_inst; head1_pred_taken=q6_pred_taken; head1_pred_nextpc=q6_pred_nextpc; head1_slot0_dual_ok=q6_slot0_dual_ok; end
        default: begin head1_pc=q7_pc; head1_inst=q7_inst; head1_pred_taken=q7_pred_taken; head1_pred_nextpc=q7_pred_nextpc; head1_slot0_dual_ok=q7_slot0_dual_ok; end
    endcase
end

wire head_ready  = slot_ready[head_ptr];
wire head1_ready = slot_ready[head1_ptr];
assign fs_to_ds_valid  = (queue_count != 4'd0) && head_ready;
assign fs_to_ds_bus    = {head_slot0_dual_ok, head_pred_taken, head_pred_nextpc, head_inst, head_pc};
assign fs_to_ds_valid1 = (queue_count >= 4'd2) && head_ready && head1_ready;
assign fs_to_ds_bus1   = {head1_slot0_dual_ok, head1_pred_taken, head1_pred_nextpc, head1_inst, head1_pc};
assign fs_slot1_meta   = head_slot1_meta;

wire launch_fetch = !reset && !br_taken && !req_pending && (queue_count < 4'd8);
wire launch_dual = launch_fetch && !bp_pred_taken &&
                   (nextpc_reg[3:2] != 2'd3) && (queue_count <= 4'd6)
`ifdef DISABLE_M16_DUAL_FETCH
                   && 1'b0
`endif
                   ;
wire active_req_dual = req_pending ? req_dual : launch_dual;
wire active_req_pred_taken = req_pending ? req_pred_taken : bp_pred_taken;
wire [31:0] active_req_pred_nextpc = req_pending ? req_pred_nextpc : bp_pred_nextpc;
wire active_req_pred_taken_slot1 = req_pending ? req_pred_taken_slot1 :
                                                bp_pred_taken_slot1;
wire [31:0] active_req_pred_nextpc_slot1 = req_pending ? req_pred_nextpc_slot1 :
                                                        bp_pred_nextpc_slot1;

assign inst_sram_req   = (req_pending || launch_fetch) && !reset && !br_taken;
assign inst_sram_wr    = 1'b0;
assign inst_sram_size  = 2'b10;
assign inst_sram_wstrb = 4'b0000;
assign inst_sram_addr  = req_pending ? req_pc : nextpc_reg;
assign inst_sram_wdata = 32'b0;
assign inst_sram_dual  = active_req_dual;
assign bp_query_pc     = nextpc_reg;

wire request_accept = inst_sram_req && inst_sram_addr_ok;
wire response_fire = inst_sram_data_ok;
wire response_is_old = response_fire && (discard_count != 3'd0);
wire response_is_live = response_fire && (discard_count == 3'd0) &&
                        (live_pending_count != 3'd0);
reg response_dual;
always @(*) begin
    case (resp_ptr)
        3'd0: response_dual = q0_req_dual;
        3'd1: response_dual = q1_req_dual;
        3'd2: response_dual = q2_req_dual;
        3'd3: response_dual = q3_req_dual;
        3'd4: response_dual = q4_req_dual;
        3'd5: response_dual = q5_req_dual;
        3'd6: response_dual = q6_req_dual;
        default: response_dual = q7_req_dual;
    endcase
end

wire [3:0] request_width = active_req_dual ? 4'd2 : 4'd1;
wire [3:0] response_width = response_dual ? 4'd2 : 4'd1;
wire [3:0] pop_width = fs_pop_base ? (fs_pop_extra ? 4'd2 : 4'd1) : 4'd0;
reg [4:0] queue_count_math;
always @(*) begin
    queue_count_math = {1'b0, queue_count};
    if (request_accept) queue_count_math = queue_count_math + {1'b0, request_width};
    if (fs_pop_base)    queue_count_math = queue_count_math - {1'b0, pop_width};
end

integer i;
always @(posedge clk) begin
    if (reset) begin
        q0_pc<=0; q0_inst<=0; q0_pred_taken<=0; q0_pred_nextpc<=0; q0_slot1_meta<=0; q0_slot0_dual_ok<=0; q0_req_dual<=0;
        q1_pc<=0; q1_inst<=0; q1_pred_taken<=0; q1_pred_nextpc<=0; q1_slot1_meta<=0; q1_slot0_dual_ok<=0; q1_req_dual<=0;
        q2_pc<=0; q2_inst<=0; q2_pred_taken<=0; q2_pred_nextpc<=0; q2_slot1_meta<=0; q2_slot0_dual_ok<=0; q2_req_dual<=0;
        q3_pc<=0; q3_inst<=0; q3_pred_taken<=0; q3_pred_nextpc<=0; q3_slot1_meta<=0; q3_slot0_dual_ok<=0; q3_req_dual<=0;
        q4_pc<=0; q4_inst<=0; q4_pred_taken<=0; q4_pred_nextpc<=0; q4_slot1_meta<=0; q4_slot0_dual_ok<=0; q4_req_dual<=0;
        q5_pc<=0; q5_inst<=0; q5_pred_taken<=0; q5_pred_nextpc<=0; q5_slot1_meta<=0; q5_slot0_dual_ok<=0; q5_req_dual<=0;
        q6_pc<=0; q6_inst<=0; q6_pred_taken<=0; q6_pred_nextpc<=0; q6_slot1_meta<=0; q6_slot0_dual_ok<=0; q6_req_dual<=0;
        q7_pc<=0; q7_inst<=0; q7_pred_taken<=0; q7_pred_nextpc<=0; q7_slot1_meta<=0; q7_slot0_dual_ok<=0; q7_req_dual<=0;
        slot_ready<=0; head_ptr<=0; tail_ptr<=0; resp_ptr<=0; queue_count<=0;
        live_pending_count<=0; req_pending<=0; req_dual<=0; req_pc<=0;
        req_pred_taken<=0; req_pred_nextpc<=0;
        req_pred_taken_slot1<=0; req_pred_nextpc_slot1<=0;
        nextpc_reg<=32'h1c00_0000;
        discard_count<=0;
    end else if (br_taken) begin
        discard_count <= discard_count - {3'b0, response_is_old} +
                         live_pending_count - {3'b0, response_is_live};
        slot_ready<=0; head_ptr<=0; tail_ptr<=0; resp_ptr<=0; queue_count<=0;
        live_pending_count<=0; req_pending<=0; req_dual<=0; nextpc_reg<=br_target;
    end else begin
        if (fs_pop_base)
            head_ptr <= head_ptr + (fs_pop_extra ? 3'd2 : 3'd1);

        if (response_is_old) begin
            discard_count <= discard_count - 4'd1;
        end else if (response_is_live) begin
            case (resp_ptr)
                3'd0: begin q0_inst<=inst_sram_rdata; q0_slot1_meta<=response_slot1_meta0; q0_slot0_dual_ok<=response_slot0_dual_ok0; slot_ready[0]<=1'b1; end
                3'd1: begin q1_inst<=inst_sram_rdata; q1_slot1_meta<=response_slot1_meta0; q1_slot0_dual_ok<=response_slot0_dual_ok0; slot_ready[1]<=1'b1; end
                3'd2: begin q2_inst<=inst_sram_rdata; q2_slot1_meta<=response_slot1_meta0; q2_slot0_dual_ok<=response_slot0_dual_ok0; slot_ready[2]<=1'b1; end
                3'd3: begin q3_inst<=inst_sram_rdata; q3_slot1_meta<=response_slot1_meta0; q3_slot0_dual_ok<=response_slot0_dual_ok0; slot_ready[3]<=1'b1; end
                3'd4: begin q4_inst<=inst_sram_rdata; q4_slot1_meta<=response_slot1_meta0; q4_slot0_dual_ok<=response_slot0_dual_ok0; slot_ready[4]<=1'b1; end
                3'd5: begin q5_inst<=inst_sram_rdata; q5_slot1_meta<=response_slot1_meta0; q5_slot0_dual_ok<=response_slot0_dual_ok0; slot_ready[5]<=1'b1; end
                3'd6: begin q6_inst<=inst_sram_rdata; q6_slot1_meta<=response_slot1_meta0; q6_slot0_dual_ok<=response_slot0_dual_ok0; slot_ready[6]<=1'b1; end
                default: begin q7_inst<=inst_sram_rdata; q7_slot1_meta<=response_slot1_meta0; q7_slot0_dual_ok<=response_slot0_dual_ok0; slot_ready[7]<=1'b1; end
            endcase
            if (response_dual) begin
                case (resp_ptr + 3'd1)
                    3'd0: begin q0_inst<=inst_sram_rdata2; q0_slot1_meta<=response_slot1_meta1; q0_slot0_dual_ok<=response_slot0_dual_ok1; slot_ready[0]<=1'b1; end
                    3'd1: begin q1_inst<=inst_sram_rdata2; q1_slot1_meta<=response_slot1_meta1; q1_slot0_dual_ok<=response_slot0_dual_ok1; slot_ready[1]<=1'b1; end
                    3'd2: begin q2_inst<=inst_sram_rdata2; q2_slot1_meta<=response_slot1_meta1; q2_slot0_dual_ok<=response_slot0_dual_ok1; slot_ready[2]<=1'b1; end
                    3'd3: begin q3_inst<=inst_sram_rdata2; q3_slot1_meta<=response_slot1_meta1; q3_slot0_dual_ok<=response_slot0_dual_ok1; slot_ready[3]<=1'b1; end
                    3'd4: begin q4_inst<=inst_sram_rdata2; q4_slot1_meta<=response_slot1_meta1; q4_slot0_dual_ok<=response_slot0_dual_ok1; slot_ready[4]<=1'b1; end
                    3'd5: begin q5_inst<=inst_sram_rdata2; q5_slot1_meta<=response_slot1_meta1; q5_slot0_dual_ok<=response_slot0_dual_ok1; slot_ready[5]<=1'b1; end
                    3'd6: begin q6_inst<=inst_sram_rdata2; q6_slot1_meta<=response_slot1_meta1; q6_slot0_dual_ok<=response_slot0_dual_ok1; slot_ready[6]<=1'b1; end
                    default: begin q7_inst<=inst_sram_rdata2; q7_slot1_meta<=response_slot1_meta1; q7_slot0_dual_ok<=response_slot0_dual_ok1; slot_ready[7]<=1'b1; end
                endcase
            end
            resp_ptr <= resp_ptr + (response_dual ? 3'd2 : 3'd1);
        end

        if (request_accept) begin
            case (tail_ptr)
                3'd0: begin q0_pc<=inst_sram_addr; q0_pred_taken<=active_req_pred_taken; q0_pred_nextpc<=active_req_pred_nextpc; q0_req_dual<=active_req_dual; slot_ready[0]<=1'b0; end
                3'd1: begin q1_pc<=inst_sram_addr; q1_pred_taken<=active_req_pred_taken; q1_pred_nextpc<=active_req_pred_nextpc; q1_req_dual<=active_req_dual; slot_ready[1]<=1'b0; end
                3'd2: begin q2_pc<=inst_sram_addr; q2_pred_taken<=active_req_pred_taken; q2_pred_nextpc<=active_req_pred_nextpc; q2_req_dual<=active_req_dual; slot_ready[2]<=1'b0; end
                3'd3: begin q3_pc<=inst_sram_addr; q3_pred_taken<=active_req_pred_taken; q3_pred_nextpc<=active_req_pred_nextpc; q3_req_dual<=active_req_dual; slot_ready[3]<=1'b0; end
                3'd4: begin q4_pc<=inst_sram_addr; q4_pred_taken<=active_req_pred_taken; q4_pred_nextpc<=active_req_pred_nextpc; q4_req_dual<=active_req_dual; slot_ready[4]<=1'b0; end
                3'd5: begin q5_pc<=inst_sram_addr; q5_pred_taken<=active_req_pred_taken; q5_pred_nextpc<=active_req_pred_nextpc; q5_req_dual<=active_req_dual; slot_ready[5]<=1'b0; end
                3'd6: begin q6_pc<=inst_sram_addr; q6_pred_taken<=active_req_pred_taken; q6_pred_nextpc<=active_req_pred_nextpc; q6_req_dual<=active_req_dual; slot_ready[6]<=1'b0; end
                default: begin q7_pc<=inst_sram_addr; q7_pred_taken<=active_req_pred_taken; q7_pred_nextpc<=active_req_pred_nextpc; q7_req_dual<=active_req_dual; slot_ready[7]<=1'b0; end
            endcase
            if (active_req_dual) begin
                case (tail_ptr + 3'd1)
                    3'd0: begin q0_pc<=inst_sram_addr+32'd4; q0_pred_taken<=active_req_pred_taken_slot1; q0_pred_nextpc<=active_req_pred_nextpc_slot1; q0_req_dual<=1'b0; slot_ready[0]<=1'b0; end
                    3'd1: begin q1_pc<=inst_sram_addr+32'd4; q1_pred_taken<=active_req_pred_taken_slot1; q1_pred_nextpc<=active_req_pred_nextpc_slot1; q1_req_dual<=1'b0; slot_ready[1]<=1'b0; end
                    3'd2: begin q2_pc<=inst_sram_addr+32'd4; q2_pred_taken<=active_req_pred_taken_slot1; q2_pred_nextpc<=active_req_pred_nextpc_slot1; q2_req_dual<=1'b0; slot_ready[2]<=1'b0; end
                    3'd3: begin q3_pc<=inst_sram_addr+32'd4; q3_pred_taken<=active_req_pred_taken_slot1; q3_pred_nextpc<=active_req_pred_nextpc_slot1; q3_req_dual<=1'b0; slot_ready[3]<=1'b0; end
                    3'd4: begin q4_pc<=inst_sram_addr+32'd4; q4_pred_taken<=active_req_pred_taken_slot1; q4_pred_nextpc<=active_req_pred_nextpc_slot1; q4_req_dual<=1'b0; slot_ready[4]<=1'b0; end
                    3'd5: begin q5_pc<=inst_sram_addr+32'd4; q5_pred_taken<=active_req_pred_taken_slot1; q5_pred_nextpc<=active_req_pred_nextpc_slot1; q5_req_dual<=1'b0; slot_ready[5]<=1'b0; end
                    3'd6: begin q6_pc<=inst_sram_addr+32'd4; q6_pred_taken<=active_req_pred_taken_slot1; q6_pred_nextpc<=active_req_pred_nextpc_slot1; q6_req_dual<=1'b0; slot_ready[6]<=1'b0; end
                    default: begin q7_pc<=inst_sram_addr+32'd4; q7_pred_taken<=active_req_pred_taken_slot1; q7_pred_nextpc<=active_req_pred_nextpc_slot1; q7_req_dual<=1'b0; slot_ready[7]<=1'b0; end
                endcase
            end
            tail_ptr <= tail_ptr + (active_req_dual ? 3'd2 : 3'd1);
            req_pending <= 1'b0;
            req_dual <= 1'b0;
            nextpc_reg <= active_req_dual ? active_req_pred_nextpc_slot1 :
                                             active_req_pred_nextpc;
        end else if (launch_fetch && !inst_sram_addr_ok) begin
            req_pending<=1'b1; req_pc<=nextpc_reg; req_pred_taken<=bp_pred_taken;
            req_pred_nextpc<=bp_pred_nextpc;
            req_pred_taken_slot1<=bp_pred_taken_slot1;
            req_pred_nextpc_slot1<=bp_pred_nextpc_slot1;
            req_dual<=launch_dual;
        end

        queue_count <= queue_count_math[3:0];
        case ({request_accept, response_is_live})
            2'b10: live_pending_count <= live_pending_count + 4'd1;
            2'b01: live_pending_count <= live_pending_count - 4'd1;
            default: live_pending_count <= live_pending_count;
        endcase
    end
end

wire unused_br_stall = br_stall;
wire unused_ds_allowin = ds_allowin;
endmodule

module id_stage(
    input                          clk           ,
    input                          reset         ,
    input  [ 7:0]                  ext_int       ,
    //allowin
    input                          es_allowin    ,
    output                         ds_allowin    ,
    //from fs
    input                          fs_to_ds_valid,
    input  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus  ,
    input                          fs_to_ds_valid1,
    input  [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus1 ,
    input  [`SLOT1_META_WD -1:0]   fs_slot1_meta ,
    output                         fs_pop_base    ,
    output                         fs_pop_extra   ,
    // restricted Slot-1 issue payload
    output                         slot1_issue_valid,
    output [11:0]                  slot1_issue_alu_op,
    output [31:0]                  slot1_issue_src1,
    output [31:0]                  slot1_issue_src2,
    output [ 4:0]                  slot1_issue_dest,
    output [31:0]                  slot1_issue_pc,
    output [31:0]                  slot1_issue_inst,
    output                         slot1_issue_is_older,
    output                         slot1_issue_src1_from_main_ms,
    output                         slot1_issue_src2_from_main_ms,
    output                         slot1_issue_src1_from_main_ws,
    output                         slot1_issue_src2_from_main_ws,
    output                         slot1_issue_src1_from_side_ms,
    output                         slot1_issue_src2_from_side_ms,
    //to es
    output                         ds_to_es_valid,
    output [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to fs / registered recovery boundary
    output                         id_br_stall             ,
    output                         id_normal_redirect_valid,
    output                         id_normal_actual_taken  ,
    output [31:0]                  id_normal_taken_target  ,
    output [31:0]                  id_normal_fallthrough   ,
    output                         id_sys_redirect_valid   ,
    output [31:0]                  id_sys_redirect_target  ,
    input                          external_flush          ,
    // resolved control-transfer update to branch predictor
    output                         bp_update_en       ,
    output [31:0]                  bp_update_pc       ,
    output                         bp_update_is_cond  ,
    output                         bp_update_taken    ,
    output [31:0]                  bp_update_target   ,
    output                         bp_update_is_call    ,
    output                         bp_update_is_return  ,
    output                         bp_update_is_indirect,
    // performance-counter sideband
    output                         perf_load_stall_o,
    output                         perf_ms_load_stall_o,
    output                         perf_branch_src_stall_o,
    output                         perf_es_raw_stall_o,
    output                         perf_mem_addr_es_stall_o,
    output                         perf_sys_stall_o,
    output                         perf_tlb_precheck_stall_o,
    output                         perf_ds_blocked_o,
    output                         perf_dual_issue_o,
    //to rf: for write back
    input  [`WS_TO_RF_BUS_WD -1:0] ws_to_rf_bus,
    //from exe/mem/wb: RAW hazard information
    input  [ 4:0]                  es_to_ds_dest,
    input  [ 4:0]                  ms_to_ds_dest,
    input  [ 4:0]                  ws_to_ds_dest,
    input                          es_to_ds_load_op,
    input                          es_to_ds_mem_load_op,
    input                          ms_to_ds_load_op,
    input                          ms_to_ds_load_leave,
    // forwarding results from EXE/MEM/WB, priority: EXE > MEM > WB
    input  [31:0]                  es_to_ds_result,
    input  [31:0]                  ms_to_ds_result,
    input  [31:0]                  ws_to_ds_result,
    // valid bits of later stages, used to serialize CSR/exception instructions
    input                          es_stage_valid,
    input                          ms_stage_valid,
    input                          ws_stage_valid,
    // restricted Slot-1 pipeline feedback
    input  [ 4:0]                  slot1_es_dest,
    input  [ 4:0]                  slot1_ms_dest,
    input  [31:0]                  slot1_ms_result,
    input                          slot1_es_stage_valid,
    input                          slot1_ms_stage_valid,
    input                          slot1_ws_stage_valid,
    input                          slot1_wb_we,
    input  [ 4:0]                  slot1_wb_waddr,
    input  [31:0]                  slot1_wb_wdata,
    input                          ll_w_commit,
    input                          sc_w_commit,
    output                         llbit_state
);

// Raw architectural branch direction.  Round 15 exports recovery fields
// separately so the outer register boundary does not contain a target mux.
wire        br_taken_raw;
wire        br_stall;
wire        load_stall;
wire        ms_load_stall;
wire        ms_load_src_match;
wire        ms_load_rj_match;
wire        ms_load_rk_match;
wire        ms_load_rd_match;
wire        ms_load_mem_addr_dep;
wire        ms_load_release_safe;
wire        ctrl_transfer_inst;
wire        bp_mispredict;
wire [31:0] br_actual_target;
wire [31:0] br_fallthrough_pc;
wire [31:0] br_rj_value;
wire [31:0] br_rd_value;
wire        br_cmp_inst;
wire        br_rj_need;
wire        br_rd_need;
wire        br_rj_es_match;
wire        br_rd_es_match;
wire        br_rj_ms_match;
wire        br_rd_ms_match;
wire        br_rj_ws_match;
wire        br_rd_ws_match;
wire        br_rj_es_raw_match;
wire        br_rd_es_raw_match;
wire        br_rj_ms_raw_match;
wire        br_rd_ms_raw_match;
wire        br_rj_ws_raw_match;
wire        br_rd_ws_raw_match;
wire        br_rj_slot1_ms_raw_match;
wire        br_rd_slot1_ms_raw_match;
wire        deferred_load_branch_candidate;
wire        deferred_load_branch_es_match;
wire        deferred_load_branch_mem_conflict;
wire        deferred_mem_load_branch_candidate;

wire        src_no_rj;
wire        src_no_rk;
wire        src_no_rd;
wire        rj_wait;
wire        rk_wait;
wire        rd_wait;
wire        no_wait;
wire        rj_eq_rd;

wire [31:0] ds_pc;
wire [31:0] ds_inst;
// Snapshot of the prediction made when this instruction was fetched.
wire        ds_pred_taken;
wire [31:0] ds_pred_nextpc;

reg         ds_valid   ;
wire        ds_ready_go;

wire [11:0] alu_op;
wire [ 6:0] muldiv_op;

wire        res_from_csr;
wire [31:0] csr_rvalue;
wire [13:0] csr_num;
wire        csr_we;
wire [31:0] csr_wmask;
wire [31:0] csr_wvalue;
wire [31:0] csr_wdata;
wire        inst_csr;
wire        inst_csrrd;
wire        inst_csrwr;
wire        inst_csrxchg;
wire        inst_syscall;
wire        inst_ertn;
wire        inst_brk;
wire        inst_rdcntid_w;
wire        inst_rdcntvl_w;
wire        inst_rdcntvh_w;
wire        inst_cpucfg;
wire        inst_tlbsrch;
wire        inst_tlbrd;
wire        inst_tlbwr;
wire        inst_tlbfill;
wire        inst_invtlb;
wire [ 4:0] invtlb_op;
wire        inst_invtlb_valid_op;
wire        inst_cacop;
    wire        inst_idle;
wire        inst_ll_w;
wire        inst_sc_w;
wire        inst_valid;
wire        sys_inst;
wire        sys_inst_no_exc;
wire        sys_stall;
    wire        idle_wait;
wire        idle_exception_pending;
wire        sys_src_es_stall;
wire        branch_src_es_stall;
wire        branch_src_ms_stall;
wire        branch_src_ws_stall;
wire        branch_src_stall;
wire        sys_taken;
wire        tlb_local_exc;
wire        tlb_side_ready;
// exp19 opt6: declare local TLB exception wires before they are used by tlb_local_exc.
wire        exc_tlbr_i;
wire        exc_pif;
wire        exc_ppi_i;

wire        exc_int;
wire        exc_adef;
wire        exc_ale;
wire        exc_ale_now;
wire [ 1:0] mem_addr_low2;
wire        exc_sys;
wire        exc_brk;
wire        exc_ine;
(* max_fanout = 8 *) wire        exc_taken;
// Branch-only exception qualification.  A decoded control transfer cannot also
// be a data access/syscall/illegal instruction, so predictor recovery/training
// need only observe interrupt, fetch-address and instruction-TLB exceptions.
wire        bp_inst_tlb_exception;
wire        bp_update_exception;
wire [ 5:0] exc_ecode;
wire [ 8:0] exc_esubcode;
wire [31:0] exc_badv;
wire [31:0] mem_addr_id;
wire [31:0] rdcnt_result;
wire [31:0] cpucfg_result;

wire [ 7:0] mem_op;
wire        load_op;
wire        src1_is_pc;
wire        src2_is_imm;
wire        res_from_mem;
wire        dst_is_r1;
wire        gr_we;
wire        mem_we;
wire        src_reg_is_rd;
wire [4: 0] dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;
wire [31:0] br_offs;
wire [31:0] jirl_offs;

wire [ 5:0] op_31_26;
wire [ 3:0] op_25_22;
wire [ 1:0] op_21_20;
wire [ 4:0] op_19_15;
wire [ 4:0] rd;
wire [ 4:0] rj;
wire [ 4:0] rk;
wire [11:0] i12;
wire [19:0] i20;
wire [15:0] i16;
wire [25:0] i26;
wire [31:0] llsc_imm;
wire [63:0] op_31_26_d;
wire [15:0] op_25_22_d;
wire [ 3:0] op_21_20_d;
wire [31:0] op_19_15_d;

wire        inst_add_w;
wire        inst_sub_w;
wire        inst_slt;
wire        inst_sltu;
wire        inst_nor;
wire        inst_and;
wire        inst_or;
wire        inst_xor;
wire        inst_slli_w;
wire        inst_srli_w;
wire        inst_srai_w;
wire        inst_addi_w;
wire        inst_ld_b;
wire        inst_ld_h;
wire        inst_ld_w;
wire        inst_ld_bu;
wire        inst_ld_hu;
wire        inst_st_b;
wire        inst_st_h;
wire        inst_st_w;
wire        inst_jirl;
wire        inst_b;
wire        inst_bl;
wire        inst_beq;
wire        inst_bne;
wire        inst_blt;
wire        inst_bge;
wire        inst_bltu;
wire        inst_bgeu;
wire        inst_lu12i_w;
wire        inst_slti;
wire        inst_sltui;
wire        inst_andi;
wire        inst_ori;
wire        inst_xori;
wire        inst_sll_w;
wire        inst_srl_w;
wire        inst_sra_w;
wire        inst_pcaddu12i;
wire        inst_mul_w;
wire        inst_mulh_w;
wire        inst_mulh_wu;
wire        inst_div_w;
wire        inst_mod_w;
wire        inst_div_wu;
wire        inst_mod_wu;
// exp12: CSR and system call instructions
// inst_csr/inst_csrrd/inst_csrwr/inst_csrxchg are declared above together with CSR control wires.

wire        need_ui5;
wire        need_si12;
wire        need_ui12;
wire        need_si16;
wire        need_si20;
wire        need_si26;
wire        src2_is_4;

wire [ 4:0] rf_raddr1;
wire [31:0] rf_rdata1;
wire [ 4:0] rf_raddr2;
wire [31:0] rf_rdata2;
wire [31:0] br_rf_rdata1;
wire [31:0] br_rf_rdata2;

wire        rf_we   ;
wire [ 4:0] rf_waddr;
wire [31:0] rf_wdata;

wire [31:0] alu_src1   ;
wire [31:0] alu_src2   ;
wire [31:0] alu_result ;

wire [31:0] mem_result;
wire [31:0] final_result;

// -------------------------------------------------------------------------
// Slot-1 candidate decode. While Slot 0 is resident in ID, fs_to_ds_bus is
// the next instruction at the IF FIFO head. If the pair is legal, that head
// enters the side lane and fs_to_ds_bus1 refills Slot 0.
// -------------------------------------------------------------------------
wire        slot1_candidate_slot0_dual_ok;
wire        slot1_candidate_pred_taken;
wire [31:0] slot1_candidate_pred_nextpc;
wire [31:0] slot1_candidate_inst;
wire [31:0] slot1_candidate_pc;
wire [ 4:0] slot1_rd;
wire [ 4:0] slot1_rj;
wire [ 4:0] slot1_rk;
wire [11:0] slot1_i12;
wire [19:0] slot1_i20;
wire        slot1_candidate_simple;
wire        slot1_candidate_mem;
wire        slot1_candidate_load;
wire        slot1_candidate_store;
wire        slot1_src1_used;
wire        slot1_src2_used;
wire        slot1_src2_is_rd;
wire        slot1_src1_is_pc;
wire [ 1:0] slot1_imm_sel;
wire [11:0] slot1_predecoded_alu_op;
wire [ 7:0] slot1_predecoded_mem_op;
wire        slot1_src1_main_es_match;
wire        slot1_src2_main_es_match;
wire        slot1_src1_side_es_match;
wire        slot1_src2_side_es_match;
wire        slot1_src1_es_hazard;
wire        slot1_src2_es_hazard;
wire        slot1_src1_ms_load_match;
wire        slot1_src2_ms_load_match;
wire        slot1_src1_ms_load_hazard;
wire        slot1_src2_ms_load_hazard;
wire        slot1_pair_raw;
wire        slot1_pair_waw;
wire        slot1_pair_legal;
wire        slot0_dual_eligible;
wire        ds_slot0_dual_static_ok;
wire        pair_normal_fire;
wire        pair_swap_mem_legal;
wire        pair_swap_mul_legal;
wire        pair_swap_legal;
wire        pair_swap_mem_fire;
wire        pair_swap_mul_fire;
wire        pair_swap_fire;
wire        pair_branch_legal;
wire        pair_branch_fire;
wire        dual_pair_fire;
wire        dual_pair_preselect;
wire        pair_refill_select;
wire        pair_interrupt_pending;
wire        pair_exception_pending;
wire        pair_issue_ready;
wire        current_simple_alu;
wire        swap_candidate_es_load_hazard;
wire        swap_candidate_side_es_hazard;
wire        swap_candidate_ms_load_hazard;
wire        swap_pair_raw;
wire        swap_pair_waw;
wire [31:0] swap_candidate_base;
wire [31:0] swap_candidate_store_data;
wire [31:0] swap_candidate_addr;
wire        swap_candidate_aligned;
wire        swap_candidate_dmw0_hit;
wire        swap_candidate_dmw1_hit;
wire        swap_candidate_dmw_hit;
wire [31:0] swap_candidate_dmw_paddr;
wire [ 1:0] swap_candidate_dmw_mat;
wire        swap_candidate_addrmode_ok;
wire        swap_candidate_direct_bypass;
wire        swap_candidate_uncached;
wire [31:0] swap_candidate_phy_addr;
wire        slot1_candidate_branch;
wire        slot1_candidate_mul;
wire        slot1_candidate_div;
wire        slot1_candidate_system;
wire        slot1_branch_cond;
wire        slot1_branch_direct;
wire        slot1_branch_rj_es_match;
wire        slot1_branch_rd_es_match;
wire        slot1_branch_rj_side_es_match;
wire        slot1_branch_rd_side_es_match;
wire        slot1_branch_rj_ms_match;
wire        slot1_branch_rd_ms_match;
wire        slot1_branch_rj_side_ms_match;
wire        slot1_branch_rd_side_ms_match;
wire        slot1_branch_pipeline_hazard;
wire        slot1_branch_pair_raw;
wire [31:0] slot1_branch_rj_value;
wire [31:0] slot1_branch_rd_value;
wire        slot1_branch_taken_raw;
wire [31:0] slot1_branch_offset;
wire [31:0] slot1_branch_target;
wire [31:0] slot1_branch_fallthrough;
wire        slot1_branch_direction_miss;
wire        slot1_branch_target_miss;
wire        slot1_branch_mispredict;
wire        swap_rj_from_next_ms;
wire        swap_rd_from_next_ms;
wire        slot1_mul_w;
wire        slot1_mulh_w;
wire        slot1_mulh_wu;
wire [ 6:0] slot1_muldiv_op;
wire        mul_swap_src1_main_es_match;
wire        mul_swap_src2_main_es_match;
wire        mul_swap_src1_side_es_match;
wire        mul_swap_src2_side_es_match;
wire        mul_swap_src1_ms_load_match;
wire        mul_swap_src2_ms_load_match;
wire        mul_swap_es_load_hazard;
wire        mul_swap_side_es_hazard;
wire        mul_swap_ms_load_hazard;
wire        mul_swap_pair_raw;
wire        mul_swap_pair_waw;
wire        mul_swap_rj_from_next_ms;
wire        mul_swap_rk_from_next_ms;
wire        mul_swap_rj_from_next_ws;
wire        mul_swap_rk_from_next_ws;
wire [31:0] slot1_rf_rdata1;
wire [31:0] slot1_rf_rdata2;
wire [31:0] slot1_rj_value;
wire [31:0] slot1_rk_value;
wire [ 4:0] slot1_src2_reg;
wire [31:0] slot1_imm;

assign {slot1_candidate_slot0_dual_ok,
        slot1_candidate_pred_taken,
        slot1_candidate_pred_nextpc,
        slot1_candidate_inst,
        slot1_candidate_pc} = fs_to_ds_bus;

assign slot1_rd  = slot1_candidate_inst[ 4: 0];
assign slot1_rj  = slot1_candidate_inst[ 9: 5];
assign slot1_rk  = slot1_candidate_inst[14:10];
assign slot1_i12 = slot1_candidate_inst[21:10];
assign slot1_i20 = slot1_candidate_inst[24: 5];

// Simulation diagnostics and subsequent issue-width work need to know which
// unsupported class occupies the younger slot.  These compact comparisons do
// not participate in functional pair admission.
assign slot1_candidate_branch = (slot1_candidate_inst[31:26] >= 6'h13) &&
                                (slot1_candidate_inst[31:26] <= 6'h1b);
assign slot1_branch_direct = (slot1_candidate_inst[31:26] == 6'h14);
assign slot1_branch_cond   = (slot1_candidate_inst[31:26] >= 6'h16) &&
                             (slot1_candidate_inst[31:26] <= 6'h1b);
assign slot1_mul_w = (slot1_candidate_inst[31:26] == 6'h00) &&
                     (slot1_candidate_inst[25:22] == 4'h0) &&
                     (slot1_candidate_inst[21:20] == 2'h1) &&
                     (slot1_candidate_inst[19:15] == 5'h18);
assign slot1_mulh_w = (slot1_candidate_inst[31:26] == 6'h00) &&
                      (slot1_candidate_inst[25:22] == 4'h0) &&
                      (slot1_candidate_inst[21:20] == 2'h1) &&
                      (slot1_candidate_inst[19:15] == 5'h19);
assign slot1_mulh_wu = (slot1_candidate_inst[31:26] == 6'h00) &&
                       (slot1_candidate_inst[25:22] == 4'h0) &&
                       (slot1_candidate_inst[21:20] == 2'h1) &&
                       (slot1_candidate_inst[19:15] == 5'h1a);
assign slot1_candidate_mul = slot1_mul_w | slot1_mulh_w | slot1_mulh_wu;
assign slot1_muldiv_op = {4'b0, slot1_mulh_wu, slot1_mulh_w, slot1_mul_w};
assign slot1_candidate_div = (slot1_candidate_inst[31:26] == 6'h00) &&
                             (slot1_candidate_inst[25:22] == 4'h0) &&
                             (slot1_candidate_inst[21:20] == 2'h2) &&
                             (slot1_candidate_inst[19:15] <= 5'h03);
assign slot1_candidate_system =
       ((slot1_candidate_inst[31:26] == 6'h01) &&
        ((slot1_candidate_inst[25:22] == 4'h0) ||
         (slot1_candidate_inst[25:22] == 4'h8) ||
         (slot1_candidate_inst[25:22] == 4'h9))) ||
       (slot1_candidate_inst == 32'h06483800) ||
       (slot1_candidate_inst == 32'h06488000) ||
       (slot1_candidate_inst[31:24] == 8'h20) ||
       (slot1_candidate_inst[31:24] == 8'h21);

// Registered IF-queue metadata removes the complete Slot-1 opcode decoder from
// the ID pair-admission cone.  The raw instruction remains available for
// operands, immediate fields and Difftest reporting.
assign {slot1_candidate_simple,
        slot1_candidate_mem,
        slot1_candidate_load,
        slot1_candidate_store,
        slot1_src1_used,
        slot1_src2_used,
        slot1_src2_is_rd,
        slot1_src1_is_pc,
        slot1_imm_sel,
        slot1_predecoded_alu_op,
        slot1_predecoded_mem_op} = fs_slot1_meta;
// Conditional branches encode their second compare operand in rd.  Reuse the
// existing fourth register-file copy for that operand while the candidate is a
// branch; ordinary ALU/store candidates retain their rk/rd selection.
assign slot1_src2_reg = slot1_branch_cond ? slot1_rd :
                        slot1_src2_is_rd ? slot1_rd : slot1_rk;

assign op_31_26  = ds_inst[31:26];
assign op_25_22  = ds_inst[25:22];
assign op_21_20  = ds_inst[21:20];
assign op_19_15  = ds_inst[19:15];

assign rd   = ds_inst[ 4: 0];
assign rj   = ds_inst[ 9: 5];
assign rk   = ds_inst[14:10];

assign i12  = ds_inst[21:10];
assign i20  = ds_inst[24: 5];
assign i16  = ds_inst[25:10];
assign i26  = {ds_inst[ 9: 0], ds_inst[25:10]};

decoder_6_64 u_dec0(.in(op_31_26 ), .out(op_31_26_d ));
decoder_4_16 u_dec1(.in(op_25_22 ), .out(op_25_22_d ));
decoder_2_4  u_dec2(.in(op_21_20 ), .out(op_21_20_d ));
decoder_5_32 u_dec3(.in(op_19_15 ), .out(op_19_15_d ));

assign inst_add_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h00];
assign inst_sub_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h02];
assign inst_slt    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h04];
assign inst_sltu   = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h05];
assign inst_nor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h08];
assign inst_and    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h09];
assign inst_or     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0a];
assign inst_xor    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0b];
assign inst_slli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h01];
assign inst_srli_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h09];
assign inst_srai_w = op_31_26_d[6'h00] & op_25_22_d[4'h1] & op_21_20_d[2'h0] & op_19_15_d[5'h11];
assign inst_addi_w = op_31_26_d[6'h00] & op_25_22_d[4'ha];
assign inst_ld_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h0];
assign inst_ld_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h1];
assign inst_ld_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h2];
assign inst_st_b   = op_31_26_d[6'h0a] & op_25_22_d[4'h4];
assign inst_st_h   = op_31_26_d[6'h0a] & op_25_22_d[4'h5];
assign inst_st_w   = op_31_26_d[6'h0a] & op_25_22_d[4'h6];
assign inst_ld_bu  = op_31_26_d[6'h0a] & op_25_22_d[4'h8];
assign inst_ld_hu  = op_31_26_d[6'h0a] & op_25_22_d[4'h9];
assign inst_jirl   = op_31_26_d[6'h13];
assign inst_b      = op_31_26_d[6'h14];
assign inst_bl     = op_31_26_d[6'h15];
assign inst_beq    = op_31_26_d[6'h16];
assign inst_bne    = op_31_26_d[6'h17];
assign inst_blt    = op_31_26_d[6'h18];
assign inst_bge    = op_31_26_d[6'h19];
assign inst_bltu   = op_31_26_d[6'h1a];
assign inst_bgeu   = op_31_26_d[6'h1b];
assign inst_lu12i_w= op_31_26_d[6'h05] & ~ds_inst[25];

// exp10: arithmetic/logic immediate, variable shift, pc-relative, mul/div/mod
assign inst_slti   = op_31_26_d[6'h00] & op_25_22_d[4'h8];
assign inst_sltui  = op_31_26_d[6'h00] & op_25_22_d[4'h9];
assign inst_andi   = op_31_26_d[6'h00] & op_25_22_d[4'hd];
assign inst_ori    = op_31_26_d[6'h00] & op_25_22_d[4'he];
assign inst_xori   = op_31_26_d[6'h00] & op_25_22_d[4'hf];
assign inst_sll_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0e];
assign inst_srl_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h0f];
assign inst_sra_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h10];
assign inst_pcaddu12i = op_31_26_d[6'h07] & ~ds_inst[25];
assign inst_mul_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h18];
assign inst_mulh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h19];
assign inst_mulh_wu= op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h1] & op_19_15_d[5'h1a];
assign inst_div_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h00];
assign inst_mod_w  = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h01];
assign inst_div_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h02];
assign inst_mod_wu = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h03];

// exp12/exp13: CSR, exception, counter instructions
// CSR format uses op[31:24]=8'h04, csr_num is ds_inst[23:10].
// Do NOT restrict op_19_15, because high CSR numbers such as TICLR(0x44),
// TCFG(0x41), TVAL(0x42), TID(0x40) occupy these bits.
assign inst_csr     = op_31_26_d[6'h01] & op_25_22_d[4'h0] & op_21_20_d[2'h0];
assign inst_csrrd   = inst_csr & (rj == 5'd0);
assign inst_csrwr   = inst_csr & (rj == 5'd1);
assign inst_csrxchg = inst_csr & (rj != 5'd0) & (rj != 5'd1);
assign inst_syscall = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h16];
assign inst_brk     = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h2] & op_19_15_d[5'h14];
assign inst_ertn    = (ds_inst == 32'h06483800);

// exp18: TLB maintenance instructions.
// Encodings follow the LoongArch privileged instruction definitions:
// tlbsrch=06482800, tlbrd=06482c00, tlbwr=06483000, tlbfill=06483400,
// invtlb uses opcode prefix 00000110010010011 and rd[4:0] as invtlb_op.
assign inst_tlbsrch = (ds_inst == 32'h06482800);
assign inst_tlbrd   = (ds_inst == 32'h06482c00);
assign inst_tlbwr   = (ds_inst == 32'h06483000);
assign inst_tlbfill = (ds_inst == 32'h06483400);
assign inst_invtlb  = op_31_26_d[6'h01] & op_25_22_d[4'h9] &
                      op_21_20_d[2'h0]  & op_19_15_d[5'h13];
assign invtlb_op    = rd;
// LoongArch simplified lab only defines INVTLB op 0..6.
// op 7 and above must be treated as illegal instruction (INE).
assign inst_invtlb_valid_op = inst_invtlb && (invtlb_op <= 5'd6);

// exp23: CACOP encoding is opcode prefix 0000011000, format cacop code,rj,si12.
// The low 5 bits (rd field) carry the operation code.
assign inst_cacop = op_31_26_d[6'h01] & op_25_22_d[4'h8];

    // LoongArch privileged idle instruction.
    assign inst_idle = (ds_inst == 32'h06488000);

// LLSC_STAGE61A_DONE
assign inst_ll_w = (ds_inst[31:24] == 8'h20);
assign inst_sc_w = (ds_inst[31:24] == 8'h21);
// rdcntid.w writes TID into rj, while rdcntvl.w/rdcntvh.w write counter low/high into rd.
assign inst_rdcntid_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0]
                       & op_19_15_d[5'h00] & (rk == 5'h18) & (rd == 5'h00);
assign inst_rdcntvl_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0]
                       & op_19_15_d[5'h00] & (rk == 5'h18) & (rj == 5'h00);
assign inst_rdcntvh_w = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0]
                       & op_19_15_d[5'h00] & (rk == 5'h19) & (rj == 5'h00);
assign inst_cpucfg    = op_31_26_d[6'h00] & op_25_22_d[4'h0] & op_21_20_d[2'h0]
                       & op_19_15_d[5'h00] & (rk == 5'h1b);

assign inst_valid = inst_add_w  | inst_sub_w  | inst_slt    | inst_sltu   |
                    inst_nor    | inst_and    | inst_or     | inst_xor    |
                    inst_slli_w | inst_srli_w | inst_srai_w | inst_addi_w |
                    inst_ld_b   | inst_ld_h   | inst_ld_w   | inst_ld_bu  | inst_ld_hu |
                    inst_st_b | inst_st_h | inst_st_w |
                    inst_jirl   | inst_b      | inst_bl     | inst_beq    | inst_bne |
                    inst_blt    | inst_bge    | inst_bltu   | inst_bgeu   |
                    inst_lu12i_w| inst_slti   | inst_sltui  | inst_andi   | inst_ori | inst_xori |
                    inst_sll_w  | inst_srl_w  | inst_sra_w  | inst_pcaddu12i |
                    inst_mul_w  | inst_mulh_w | inst_mulh_wu| inst_div_w  | inst_mod_w |
                    inst_div_wu | inst_mod_wu |
                    inst_csrrd  | inst_csrwr  | inst_csrxchg| inst_syscall| inst_brk | inst_ertn |
                    inst_tlbsrch| inst_tlbrd  | inst_tlbwr  | inst_tlbfill| inst_invtlb_valid_op | inst_cacop |
                    inst_rdcntid_w | inst_rdcntvl_w | inst_rdcntvh_w | inst_cpucfg | inst_idle | inst_ll_w | inst_sc_w;

assign sys_inst_no_exc = inst_csrrd | inst_csrwr | inst_csrxchg | inst_syscall | inst_brk | inst_ertn |
                         inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill | inst_invtlb_valid_op |
                         inst_rdcntid_w | inst_rdcntvl_w | inst_rdcntvh_w;
assign sys_inst = sys_inst_no_exc | exc_taken;

assign alu_op[ 0] = inst_add_w | inst_addi_w
                    | inst_ld_b | inst_ld_h | inst_ld_w | inst_ld_bu | inst_ld_hu
                    | inst_st_b | inst_st_h | inst_st_w | inst_ll_w | inst_sc_w | inst_cacop
                    | inst_jirl | inst_bl | inst_pcaddu12i;
assign alu_op[ 1] = inst_sub_w;
assign alu_op[ 2] = inst_slt | inst_slti;
assign alu_op[ 3] = inst_sltu | inst_sltui;
assign alu_op[ 4] = inst_and | inst_andi;
assign alu_op[ 5] = inst_nor;
assign alu_op[ 6] = inst_or | inst_ori;
assign alu_op[ 7] = inst_xor | inst_xori;
assign alu_op[ 8] = inst_slli_w | inst_sll_w;
assign alu_op[ 9] = inst_srli_w | inst_srl_w;
assign alu_op[10] = inst_srai_w | inst_sra_w;
assign alu_op[11] = inst_lu12i_w;

// one-hot: [0] mul.w, [1] mulh.w, [2] mulh.wu, [3] div.w, [4] mod.w, [5] div.wu, [6] mod.wu
assign muldiv_op = {inst_mod_wu, inst_div_wu, inst_mod_w, inst_div_w,
                    inst_mulh_wu, inst_mulh_w, inst_mul_w};

assign need_ui5   =  inst_slli_w | inst_srli_w | inst_srai_w;
assign need_si12  =  inst_addi_w
                   | inst_ld_b | inst_ld_h | inst_ld_w | inst_ld_bu | inst_ld_hu
                   | inst_st_b | inst_st_h | inst_st_w | inst_cacop
                   | inst_slti | inst_sltui;
assign need_ui12  =  inst_andi | inst_ori | inst_xori;
assign need_si16  =  inst_jirl | inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu;
assign need_si20  =  inst_lu12i_w | inst_pcaddu12i;
assign need_si26  =  inst_b | inst_bl;
assign src2_is_4  =  inst_jirl | inst_bl;
assign llsc_imm = {{16{ds_inst[23]}}, ds_inst[23:10], 2'b0};


assign imm = src2_is_4 ? 32'h4                      :
             need_si20 ? {i20[19:0], 12'b0}         :
             (inst_ll_w | inst_sc_w) ? llsc_imm     :
             need_ui5  ? rk                         :
             need_ui12 ? {20'b0, i12[11:0]}         :
            /*need_si12*/{{20{i12[11]}}, i12[11:0]} ;

assign br_offs = need_si26 ? {{ 4{i26[25]}}, i26[25:0], 2'b0} :
                              {{14{i16[15]}}, i16[15:0], 2'b0} ;

assign jirl_offs = {{14{i16[15]}}, i16[15:0], 2'b0};

assign src_reg_is_rd = inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu
                     | inst_st_b | inst_st_h | inst_st_w | inst_sc_w
                     | inst_csrwr | inst_csrxchg;

assign src1_is_pc    = inst_jirl | inst_bl | inst_pcaddu12i;

assign src2_is_imm   = inst_slli_w |
                       inst_srli_w |
                       inst_srai_w |
                       inst_addi_w |
                       inst_slti   |
                       inst_sltui  |
                       inst_andi   |
                       inst_ori    |
                       inst_xori   |
                       inst_ld_b   |
                       inst_ld_h   |
                       inst_ld_w   |
                       inst_ld_bu  |
                       inst_ld_hu  |
                       inst_st_b   |
                       inst_st_h   |
                       inst_st_w   |
                       inst_ll_w   |
                       inst_sc_w   |
                       inst_cacop |
                       inst_lu12i_w|
                       inst_pcaddu12i |
                       inst_jirl   |
                       inst_bl     ;

assign mem_op = {inst_st_w | inst_sc_w, inst_st_h, inst_st_b, inst_ld_hu, inst_ld_bu, inst_ld_w | inst_ll_w, inst_ld_h, inst_ld_b};
assign load_op = inst_ld_b | inst_ld_h | inst_ld_w | inst_ld_bu | inst_ld_hu | inst_ll_w;
assign res_from_mem  = load_op;
assign dst_is_r1     = inst_bl;
assign gr_we         = ~(inst_st_b | inst_st_h | inst_st_w)
                     & ~(inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu | inst_b)
                     & ~(inst_syscall | inst_brk | inst_ertn | inst_cacop | inst_idle | exc_taken)
                     & ~(inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill | inst_invtlb);
assign mem_we = inst_st_b | inst_st_h | inst_st_w | inst_sc_w;
assign dest          = inst_rdcntid_w ? rj :
                       dst_is_r1      ? 5'd1 : rd;

assign rf_raddr1 = rj;
assign rf_raddr2 = src_reg_is_rd ? rd :rk;
regfile u_regfile(
    .clk    (clk      ),
    .raddr1 (rf_raddr1),
    .rdata1 (rf_rdata1),
    .raddr2   (rf_raddr2  ),
    .rdata2   (rf_rdata2  ),
    .br_raddr1(rj          ),
    .br_rdata1(br_rf_rdata1),
    .br_raddr2(rd          ),
    .br_rdata2(br_rf_rdata2),
    .raddr3  (slot1_rj       ),
    .rdata3  (slot1_rf_rdata1),
    .raddr4  (slot1_src2_reg ),
    .rdata4  (slot1_rf_rdata2),
    .we       (rf_we       ),
    .waddr  (rf_waddr ),
    .wdata  (rf_wdata ),
    .we2    (slot1_wb_we   ),
    .waddr2 (slot1_wb_waddr),
    .wdata2 (slot1_wb_wdata)
    );

// exp19 opt17/18 cleanup: declare RAW-match wires before use.  Vivado
// otherwise creates implicit nets first and reports Synth 8-6901.
wire        rj_es_match;
wire        rj_ms_match;
wire        rj_ws_match;
wire        rk_es_match;
wire        rk_ms_match;
wire        rk_ws_match;
wire        rd_es_match;
wire        rd_ms_match;
wire        rd_ws_match;
// Round 10: encode late forwarding decisions in ID.  The instruction reaches
// EXE only when an EXE producer is guaranteed to advance to MEM, or when a
// released MEM load is guaranteed to advance to WB.  Carrying these six bits
// removes three 5-bit destination comparators from the EXE->DCache address path.
wire        rj_from_next_ms;
wire        rk_from_next_ms;
wire        rd_from_next_ms;
wire        rj_from_next_ws;
wire        rk_from_next_ws;
wire        rd_from_next_ws;
wire        es_raw_stall;
wire        mem_addr_es_stall;
wire [31:0] mem_base_value;

// Slot-1 hazard matches for the current Slot-0 instruction. A consumer waits
// while its producer is in Slot-1 EXE, then uses the registered Slot-1 MEM result.
wire rj_slot1_es_match;
wire rk_slot1_es_match;
wire rd_slot1_es_match;
wire rj_slot1_ms_match;
wire rk_slot1_ms_match;
wire rd_slot1_ms_match;
wire slot1_es_raw_stall;

// exp19 opt8:
// Structural timing cut: do not forward EXE-stage combinational results back
// into ID at all.  Also do not forward a MEM-stage load result into ID in the
// same cycle.  Load data may come directly from AXI rvalid/rdata, and forwarding
// it into ID would recreate a long path:
// AXI R channel -> MEM load result -> ID address/TLB/CSR/branch control.
// If ID needs a value that is currently produced by EXE or by a MEM load, it
// stalls and then consumes the registered WB-stage value.
assign rj_value  = rj_slot1_ms_match ? slot1_ms_result :
                   (rj_ms_match && !ms_to_ds_load_op) ? ms_to_ds_result :
                                                       rf_rdata1;

assign rkd_value = rk_slot1_ms_match ? slot1_ms_result :
                   rd_slot1_ms_match ? slot1_ms_result :
                   (rk_ms_match && !ms_to_ds_load_op) ? ms_to_ds_result :
                   (rd_ms_match && !ms_to_ds_load_op) ? ms_to_ds_result :
                                                       rf_rdata2;

// Timing optimization for exp19:
// Do not use an EXE-stage forwarded ALU result as the load/store base address in ID.
// Otherwise the path EXE ALU result -> data-side DMW/TLB search -> exception/IF control
// becomes a long same-cycle path.  When a memory base register is produced by EXE,
// ID stalls one cycle and then uses the MEM-stage registered result below.
assign rj_es_match = !src_no_rj && (rj != 5'b0) && (rj == es_to_ds_dest);
assign rj_ms_match = !src_no_rj && (rj != 5'b0) && (rj == ms_to_ds_dest);
assign rj_ws_match = !src_no_rj && (rj != 5'b0) && (rj == ws_to_ds_dest);

assign rk_es_match = !src_no_rk && (rk != 5'b0) && (rk == es_to_ds_dest);
assign rk_ms_match = !src_no_rk && (rk != 5'b0) && (rk == ms_to_ds_dest);
assign rk_ws_match = !src_no_rk && (rk != 5'b0) && (rk == ws_to_ds_dest);

assign rd_es_match = !src_no_rd && (rd != 5'b0) && (rd == es_to_ds_dest);
assign rd_ms_match = !src_no_rd && (rd != 5'b0) && (rd == ms_to_ds_dest);
assign rd_ws_match = !src_no_rd && (rd != 5'b0) && (rd == ws_to_ds_dest);

// Stage3 max-perf: ordinary EXE->ID RAW is no longer a blanket stall.
// A dependent ALU instruction is allowed to enter EXE and consumes the producer
// through MEM/WB -> EXE forwarding in the next cycle.  ID still blocks the
// cases that really need the value in ID: load/mul/div-use, memory base address,
// branch/jirl comparison/target, and CSR/TLB/system side effects.
assign es_raw_stall = 1'b0;

assign mem_base_value = rj_slot1_ms_match ? slot1_ms_result :
                        (rj_ms_match && !ms_to_ds_load_op) ? ms_to_ds_result :
                                                            rf_rdata1;

assign src_no_rj = inst_b | inst_bl | inst_lu12i_w | inst_pcaddu12i
                 | inst_csrrd | inst_csrwr | inst_syscall | inst_brk | inst_ertn
                 | inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill
                 | inst_rdcntid_w | inst_rdcntvl_w | inst_rdcntvh_w;
assign src_no_rk = inst_slli_w | inst_srli_w | inst_srai_w |
                   inst_addi_w |
                   inst_ld_b   | inst_ld_h   | inst_ld_w   | inst_ld_bu | inst_ld_hu |
                   inst_st_b   | inst_st_h   | inst_st_w   | inst_ll_w | inst_sc_w |
                   inst_slti   | inst_sltui  | inst_andi   |
                   inst_ori    | inst_xori   | inst_pcaddu12i |
                   inst_jirl   | inst_b      | inst_bl     |
                   inst_beq    | inst_bne    | inst_blt    | inst_bge | inst_bltu | inst_bgeu |
                   inst_lu12i_w | inst_csrrd  | inst_csrwr  | inst_csrxchg | inst_syscall | inst_brk | inst_ertn |
                   inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill |
                   inst_rdcntid_w | inst_rdcntvl_w | inst_rdcntvh_w | inst_cpucfg;

assign rj_slot1_es_match = !src_no_rj && (rj != 5'b0) && (rj == slot1_es_dest);
assign rk_slot1_es_match = !src_no_rk && (rk != 5'b0) && (rk == slot1_es_dest);
assign rd_slot1_es_match = !src_no_rd && (rd != 5'b0) && (rd == slot1_es_dest);
assign rj_slot1_ms_match = !src_no_rj && (rj != 5'b0) && (rj == slot1_ms_dest);
assign rk_slot1_ms_match = !src_no_rk && (rk != 5'b0) && (rk == slot1_ms_dest);
assign rd_slot1_ms_match = !src_no_rd && (rd != 5'b0) && (rd == slot1_ms_dest);
assign slot1_es_raw_stall = ds_valid &&
                            (rj_slot1_es_match | rk_slot1_es_match | rd_slot1_es_match);

// Timing optimization for exp19 opt4:
// Do not let EXE-stage forwarded values directly drive CSR/TLB side effects.
// A system/TLB/CSR instruction whose source is still in EXE waits one cycle and
// then uses the MEM-stage registered forwarding result.
assign sys_src_es_stall = ds_valid && sys_inst_no_exc &&
                          (((!src_no_rj) && (rj != 5'b0) && (rj == es_to_ds_dest)) ||
                           ((!src_no_rk) && (rk != 5'b0) && (rk == es_to_ds_dest)) ||
                           ((!src_no_rd) && (rd != 5'b0) && (rd == es_to_ds_dest)));

// Timing optimization for exp19 opt5:
// Branch/jirl targets and compare results used to feed IF redirect in the
// same cycle.  Do not let an EXE-stage forwarded result drive this path;
// stall one cycle and then use the MEM/WB registered forwarding value.
assign br_cmp_inst = inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu;
// M19.6 dynamic branch split:
// - an operand-independent conditional branch stays in ID for the earliest
//   possible predictor update;
// - a conditional branch that reads the current EXE producer enters EXE
//   immediately instead of spending one cycle stalled in ID;
// - jirl always resolves in EXE because its dynamic register target is the
//   expensive ID target path.
wire exe_deferred_ctrl_inst = inst_jirl |
                              (br_cmp_inst &&
                               (br_rj_es_match | br_rd_es_match));
assign br_rj_need  = inst_jirl | br_cmp_inst;
assign br_rd_need  = br_cmp_inst;

// M19.11 branch-compare timing cut.
//
// Form the physical register-number matches without opcode qualification and
// use those raw matches in the operand muxes.  The former br_*_need gating made
// instruction decode select the comparator inputs, so an opcode bit traversed
// decode, forwarding muxes and the full 32-bit less-than carry chain before the
// redirect register.  Opcode qualification remains on hazards/dispatch only.
assign br_rj_es_raw_match = (rj != 5'b0) && (rj == es_to_ds_dest);
assign br_rd_es_raw_match = (rd != 5'b0) && (rd == es_to_ds_dest);
assign br_rj_ms_raw_match = (rj != 5'b0) && (rj == ms_to_ds_dest);
assign br_rd_ms_raw_match = (rd != 5'b0) && (rd == ms_to_ds_dest);
assign br_rj_ws_raw_match = (rj != 5'b0) && (rj == ws_to_ds_dest);
assign br_rd_ws_raw_match = (rd != 5'b0) && (rd == ws_to_ds_dest);
assign br_rj_slot1_ms_raw_match = (rj != 5'b0) && (rj == slot1_ms_dest);
assign br_rd_slot1_ms_raw_match = (rd != 5'b0) && (rd == slot1_ms_dest);

assign br_rj_es_match = br_rj_need && br_rj_es_raw_match;
assign br_rd_es_match = br_rd_need && br_rd_es_raw_match;
assign br_rj_ms_match = br_rj_need && br_rj_ms_raw_match;
assign br_rd_ms_match = br_rd_need && br_rd_ms_raw_match;
assign br_rj_ws_match = br_rj_need && br_rj_ws_raw_match;
assign br_rd_ws_match = br_rd_need && br_rd_ws_raw_match;

// M17.8B deferred load-branch dispatch.
// Only a real main-EXE load qualifies; unfinished multiply/divide operations
// still use es_to_ds_load_op but are deliberately excluded.  A simultaneous
// older MEM load matching either branch operand is kept conservative, because
// the EXE comparator has exactly one direct load-result source and the younger
// producer priority would otherwise require another selector/comparator cone.
assign deferred_load_branch_es_match = exe_deferred_ctrl_inst &&
                                       es_to_ds_mem_load_op &&
                                       (br_rj_es_match | br_rd_es_match);
assign deferred_load_branch_mem_conflict = ms_to_ds_load_op &&
                                           (br_rj_ms_match | br_rd_ms_match) &&
                                           !ms_to_ds_load_leave;
// When the matching MEM load leaves on this edge, its formatted result will be
// resident in WB during the branch's first EXE cycle.  Dispatch the branch now
// and consume that registered value there instead of spending another cycle in
// ID.  A non-ready MEM load remains a hard conflict above.
assign deferred_mem_load_branch_candidate =
       (inst_jirl | br_cmp_inst) && ms_to_ds_load_op &&
       (br_rj_ms_match | br_rd_ms_match) && ms_to_ds_load_leave;
`ifdef DISABLE_M178B_DEFERRED_LOAD_BRANCH
assign deferred_load_branch_candidate = 1'b0;
`else
// M19.4 resolves every main-lane control transfer from registered EXE
// operands.  The old ID path combined instruction-bank selection, operand
// forwarding, a 32-bit target add/compare and redirect qualification in one
// cycle.  A matching EXE load is the only special case that waits for MEM;
// ordinary forwarded producers use the normal registered EXE selectors.
assign deferred_load_branch_candidate = ds_valid &&
                                         (exe_deferred_ctrl_inst |
                                          deferred_mem_load_branch_candidate) &&
                                         !deferred_load_branch_mem_conflict;
`endif

assign branch_src_es_stall = ds_valid && (br_rj_es_match | br_rd_es_match) &&
                             !deferred_load_branch_candidate;
// Branch operands can already be forwarded from MEM for non-loads and from WB.
// Only a MEM-stage load still needs to wait; otherwise we waste one branch cycle.
assign branch_src_ms_stall = ds_valid && ms_to_ds_load_op &&
                             (br_rj_ms_match | br_rd_ms_match) &&
                             !deferred_load_branch_candidate;
assign branch_src_ws_stall = 1'b0;
assign branch_src_stall    = branch_src_es_stall | branch_src_ms_stall;
// Branch operands use dedicated distributed-RAM copies addressed directly by
// rj/rd.  This removes src_reg_is_rd decode and the generic read-port mux from
// the branch recovery critical path.  WB bypass is already inside regfile.
assign br_rj_value = br_rj_slot1_ms_raw_match ? slot1_ms_result :
                     (br_rj_ms_raw_match && !ms_to_ds_load_op) ?
                         ms_to_ds_result : br_rf_rdata1;
assign br_rd_value = br_rd_slot1_ms_raw_match ? slot1_ms_result :
                     (br_rd_ms_raw_match && !ms_to_ds_load_op) ?
                         ms_to_ds_result : br_rf_rdata2;
// ---------------- CSR register file used by exp12/exp13 ----------------
localparam CSR_CRMD   = 14'h0000;
localparam CSR_PRMD   = 14'h0001;
localparam CSR_ECFG   = 14'h0004;
localparam CSR_ESTAT  = 14'h0005;
localparam CSR_ERA    = 14'h0006;
localparam CSR_BADV   = 14'h0007;
localparam CSR_EENTRY = 14'h000c;
localparam CSR_SAVE0  = 14'h0030;
localparam CSR_SAVE1  = 14'h0031;
localparam CSR_SAVE2  = 14'h0032;
localparam CSR_SAVE3  = 14'h0033;
localparam CSR_TID    = 14'h0040;
localparam CSR_TLBIDX = 14'h0010;
localparam CSR_TLBEHI = 14'h0011;
localparam CSR_TLBELO0= 14'h0012;
localparam CSR_TLBELO1= 14'h0013;
localparam CSR_ASID   = 14'h0018;
localparam CSR_TCFG   = 14'h0041;
localparam CSR_TVAL   = 14'h0042;
localparam CSR_TICLR  = 14'h0044;
localparam CSR_LLBCTL = 14'h0060;
localparam CSR_TLBRENTRY = 14'h0088;
localparam CSR_DMW0      = 14'h0180;
localparam CSR_DMW1      = 14'h0181;

(* max_fanout = 8 *) reg [31:0] csr_crmd;
reg [31:0] csr_prmd;
(* max_fanout = 8 *) reg [31:0] csr_ecfg;
(* max_fanout = 8 *) reg [31:0] csr_estat;
reg [31:0] csr_era;
reg [31:0] csr_badv;
reg [31:0] csr_eentry;
reg [31:0] csr_save0;
reg [31:0] csr_save1;
reg [31:0] csr_save2;
reg [31:0] csr_save3;
reg [31:0] csr_tid;
(* max_fanout = 8 *) reg [31:0] csr_tcfg;
reg [31:0] csr_llbctl;
reg [31:0] csr_tlbidx;
reg [31:0] csr_tlbehi;
reg [31:0] csr_tlbelo0;
reg [31:0] csr_tlbelo1;
reg [31:0] csr_asid;
reg [31:0] csr_tlbrentry;
(* max_fanout = 8 *) reg [31:0] csr_dmw0;
(* max_fanout = 8 *) reg [31:0] csr_dmw1;
reg [ 4:0]  tlbfill_index;
reg [31:0] timer_cnt;
reg [63:0] stable_counter;

wire [31:0] csr_tval;
wire [31:0] csr_ticlr;
wire        timer_enabled;
wire        timer_periodic;
wire [31:0] timer_init_value;
wire [ 7:0] hw_int;

// -----------------------------------------------------------------------------
// exp19 ref-opt v4: ID-local system side-effect staging.
//
// The successful reference design does not let current ID decode/TLB-search drive
// CSR/TLB write enables in the same cycle.  Keep the original pipeline interface,
// but stage exception/ERTN/TLB-maintenance information for one cycle and commit
// CSR/TLB side effects only from these registers.  Ordinary ALU/load/store/branch
// and simple CSR read/write instructions keep the original flow.
// -----------------------------------------------------------------------------
reg         sys_stage_valid;
// Dedicated low-fanout copy of the resident INVTLB state.  It is loaded and
// cleared on the same edges as sys_stage_valid, avoiding the high-fanout
// sys_stage_valid decode on the timing-critical TLB search-input muxes.
reg         sys_invtlb_active_r;
reg         sys_exc_taken_r;
reg [ 5:0]  sys_exc_ecode_r;
reg [ 8:0]  sys_exc_esubcode_r;
reg [31:0]  sys_exc_badv_r;
reg         sys_exc_tlbr_r;
reg         sys_exc_adef_r;
reg         sys_exc_ale_r;
reg         sys_exc_tlb_related_r;
reg [31:0]  sys_pc_r;
reg [31:0]  sys_inst_r;

reg         sys_inst_ertn_r;
reg         sys_inst_tlbsrch_r;
reg         sys_inst_tlbrd_r;
reg         sys_inst_tlbwr_r;
reg         sys_inst_tlbfill_r;
reg         sys_inst_invtlb_r;
reg [ 4:0]  sys_invtlb_op_r;
reg [18:0]  sys_invtlb_vppn_r;
reg         sys_invtlb_va_bit12_r;
reg [ 9:0]  sys_invtlb_asid_r;

reg         sys_tlb_s1_found_r;
reg [ 4:0]  sys_tlb_s1_index_r;
reg         sys_tlb_r_e_r;
reg [18:0]  sys_tlb_r_vppn_r;
reg [ 5:0]  sys_tlb_r_ps_r;
reg [ 9:0]  sys_tlb_r_asid_r;
reg         sys_tlb_r_g_r;
reg [19:0]  sys_tlb_r_ppn0_r;
reg [ 1:0]  sys_tlb_r_plv0_r;
reg [ 1:0]  sys_tlb_r_mat0_r;
reg         sys_tlb_r_d0_r;
reg         sys_tlb_r_v0_r;
reg [19:0]  sys_tlb_r_ppn1_r;
reg [ 1:0]  sys_tlb_r_plv1_r;
reg [ 1:0]  sys_tlb_r_mat1_r;
reg         sys_tlb_r_d1_r;
reg         sys_tlb_r_v1_r;
reg [ 4:0]  sys_tlbfill_index_r;

// Timing cut for the ID TLB/exception cone.  When the current ID entry needs
// TLB/MMU information, hold it for one precheck cycle and use these registered
// results on the following cycle to decide whether it may enter EXE.
reg         ds_tlb_checked_r;
reg         ds_tlb_lookup_r;
reg [31:0]  ds_tlb_exc_badv_r;
reg         ds_tlb_exc_tlbr_r;
reg         ds_tlb_exc_pif_r;
reg         ds_tlb_exc_pil_r;
reg         ds_tlb_exc_pis_r;
reg         ds_tlb_exc_pme_r;
reg         ds_tlb_exc_ppi_r;
reg         ds_pre_inst_need_tlb_r;
reg         ds_pre_data_need_tlb_r;
reg         ds_pre_exc_adef_r;
reg         ds_pre_exc_ale_r;
reg         ds_pre_mem_store_r;
reg         ds_pre_tlb_read_like_r;
reg         ds_pre_tlb_access_like_r;
reg         ds_pre_direct_addr_mode_r;
reg         ds_pre_data_dmw_hit_r;
reg [ 1:0]  ds_pre_crmd_plv_r;
reg [31:0]  ds_pre_pc_r;
reg [31:0]  ds_pre_mem_addr_id_r;
reg [31:0]  ds_pre_data_dmw_paddr_r;
reg [18:0]  ds_tlb_s0_vppn_r;
reg         ds_tlb_s0_va_bit12_r;
reg [ 9:0]  ds_tlb_s0_asid_r;
reg [18:0]  ds_tlb_s1_vppn_r;
reg         ds_tlb_s1_va_bit12_r;
reg [ 9:0]  ds_tlb_s1_asid_r;
reg         ds_pre_tlb_s0_found_r;
reg         ds_pre_tlb_s0_v_r;
reg         ds_pre_tlb_s0_plv_fail_r;
reg         ds_pre_tlb_s1_found_r;
reg [ 4:0]  ds_pre_tlb_s1_index_r;
reg [19:0]  ds_pre_tlb_s1_ppn_r;
reg [ 5:0]  ds_pre_tlb_s1_ps_r;
reg [ 1:0]  ds_pre_tlb_s1_mat_r;
reg         ds_pre_tlb_s1_v_r;
reg         ds_pre_tlb_s1_d_r;
reg         ds_pre_tlb_s1_plv_fail_r;
reg [31:0]  ds_mem_addr_phy_r;

// exp19 ref-local v6: stage CSR write side effects as well.
// v5 still let current ds_inst/csr_num/csr_wdata drive CSR register CE
// through normal_commit_fire, so fs_to_ds_bus_r -> csr_save*_CE remained critical.
reg         sys_csr_we_r;
reg [13:0]  sys_csr_num_r;
reg [31:0]  sys_csr_wdata_r;

wire        sys_stage_inst;
wire        sys_stage_start;
wire        sys_capture_fire;
wire        sys_commit_fire;
wire        normal_commit_fire;
wire        normal_ready_go;
wire        id_simple_ready_go;
wire        older_pipe_empty;
wire        sys_taken_commit;
wire [31:0] sys_br_target;
wire        ds_addr_precheck_needed;
wire        ds_tlb_check_needed;
wire        ds_fast_addr_check_needed;
wire        ds_precheck_needed;
wire        ds_tlb_lookup_fire;
wire        ds_tlb_classify_fire;
wire        ds_fast_addr_capture_fire;
wire        ds_tlb_precheck_stall;
wire        exc_tlb_taken_pre;
wire [ 5:0] exc_tlb_ecode_pre;
wire [31:0] exc_tlb_badv_pre;
wire        exc_tlb_taken_r;
wire [ 5:0] exc_tlb_ecode_r;
wire        exc_tlb_taken_sel;
wire [ 5:0] exc_tlb_ecode_sel;
wire [31:0] exc_tlb_badv_sel;
wire        exc_tlbr_sel;
wire        exc_tlb_related_sel;
wire        ds_registered_tlb_access;
wire [31:0] mem_addr_phy_to_es;
wire        direct_addr_exe_bypass;
wire        dmw_addr_exe_bypass;
wire [ 2:0] dmw_pseg_to_es;

wire        csr_exc_taken;
wire [ 5:0] csr_exc_ecode;
wire [ 8:0] csr_exc_esubcode;
wire [31:0] csr_exc_badv;
wire        csr_exc_tlbr;
wire        csr_exc_adef;
wire        csr_exc_ale;
wire        csr_exc_tlb_related;
wire [31:0] csr_pc;
wire        csr_inst_ertn;
wire        csr_inst_tlbsrch;
wire        csr_inst_tlbrd;
wire        csr_inst_tlbfill;
wire        csr_tlbsrch_found;
wire [ 4:0] csr_tlbsrch_index;
wire        csr_tlbrd_e;
wire [18:0] csr_tlbrd_vppn;
wire [ 5:0] csr_tlbrd_ps;
wire [ 9:0] csr_tlbrd_asid;
wire        csr_tlbrd_g;
wire [19:0] csr_tlbrd_ppn0;
wire [ 1:0] csr_tlbrd_plv0;
wire [ 1:0] csr_tlbrd_mat0;
wire        csr_tlbrd_d0;
wire        csr_tlbrd_v0;
wire [19:0] csr_tlbrd_ppn1;
wire [ 1:0] csr_tlbrd_plv1;
wire [ 1:0] csr_tlbrd_mat1;
wire        csr_tlbrd_d1;
wire        csr_tlbrd_v1;
wire        csr_we_commit;
wire [13:0] csr_num_commit;
wire [31:0] csr_wdata_commit;

wire [18:0] tlb_s1_vppn_to_tlb;
wire        tlb_s1_va_bit12_to_tlb;
wire [ 9:0] tlb_s1_asid_to_tlb;
wire [ 4:0] tlb_invtlb_op_to_tlb;


// ---------------- TLB/MMU instance for exp18/exp19 ----------------
// Port 0 searches the current instruction virtual PC for fetch translation /
// fetch-side TLB exceptions. Port 1 is shared by load/store address
// translation and TLB maintenance instructions.  TLBSRCH/INVTLB are serialized
// by sys_stall, so they do not conflict with an older memory access.
wire        tlb_s0_found;
wire [ 4:0] tlb_s0_index;
wire [19:0] tlb_s0_ppn;
wire [ 5:0] tlb_s0_ps;
wire [ 1:0] tlb_s0_plv;
wire [ 1:0] tlb_s0_mat;
wire        tlb_s0_d;
wire        tlb_s0_v;

wire        tlb_s1_found;
wire [ 4:0] tlb_s1_index;
wire [19:0] tlb_s1_ppn;
wire [ 5:0] tlb_s1_ps;
wire [ 1:0] tlb_s1_plv;
wire [ 1:0] tlb_s1_mat;
wire        tlb_s1_d;
wire        tlb_s1_v;

wire        tlb_r_e;
wire [18:0] tlb_r_vppn;
wire [ 5:0] tlb_r_ps;
wire [ 9:0] tlb_r_asid;
wire        tlb_r_g;
wire [19:0] tlb_r_ppn0;
wire [ 1:0] tlb_r_plv0;
wire [ 1:0] tlb_r_mat0;
wire        tlb_r_d0;
wire        tlb_r_v0;
wire [19:0] tlb_r_ppn1;
wire [ 1:0] tlb_r_plv1;
wire [ 1:0] tlb_r_mat1;
wire        tlb_r_d1;
wire        tlb_r_v1;

wire [ 4:0] tlb_w_index;
wire        tlb_we;
wire        tlb_w_e;
wire [18:0] tlb_w_vppn;
wire [ 5:0] tlb_w_ps;
wire [ 9:0] tlb_w_asid;
wire        tlb_w_g;
wire [19:0] tlb_w_ppn0;
wire [ 1:0] tlb_w_plv0;
wire [ 1:0] tlb_w_mat0;
wire        tlb_w_d0;
wire        tlb_w_v0;
wire [19:0] tlb_w_ppn1;
wire [ 1:0] tlb_w_plv1;
wire [ 1:0] tlb_w_mat1;
wire        tlb_w_d1;
wire        tlb_w_v1;

wire        tlb_invtlb_valid;
(* max_fanout = 8 *) wire [18:0] tlb_s1_vppn;
wire        tlb_s1_va_bit12;
wire [ 9:0] tlb_s1_asid;

wire ds_mem_load;
wire ds_mem_store;
wire ds_mem_access;
wire ds_cacheop_access;

assign ds_mem_load = inst_ld_b | inst_ld_h | inst_ld_w | inst_ld_bu | inst_ld_hu | inst_ll_w;
assign ds_mem_store = inst_st_b | inst_st_h | inst_st_w | inst_sc_w;
assign ds_mem_access = ds_mem_load | ds_mem_store;
assign ds_cacheop_access = ds_mem_access | inst_cacop;

assign mem_addr_es_stall = ds_valid && ds_mem_access && rj_es_match;

wire ds_mem_mmu_active;
wire ds_addr_mmu_active;
assign ds_mem_mmu_active = ds_mem_access && !load_stall && !ms_load_stall && !mem_addr_es_stall;
assign ds_addr_mmu_active = ds_cacheop_access && !load_stall && !ms_load_stall && !mem_addr_es_stall;

assign tlb_s1_vppn     = inst_invtlb       ? rkd_value[31:13] :
                         ds_addr_mmu_active ? mem_addr_id[31:13] : csr_tlbehi[31:13];
assign tlb_s1_va_bit12 = inst_invtlb       ? rkd_value[12] :
                         ds_addr_mmu_active ? mem_addr_id[12] : 1'b0;
assign tlb_s1_asid     = inst_invtlb ? rj_value[9:0] : csr_asid[9:0];

// Current-cycle s1 values are still used for load/store translation and for the
// first-cycle capture of INVTLB operands.  The real INVTLB side effect is driven
// from registered operands in the second cycle.
assign tlb_s1_vppn_to_tlb     = sys_invtlb_active_r ? sys_invtlb_vppn_r     : ds_tlb_s1_vppn_r;
assign tlb_s1_va_bit12_to_tlb = sys_invtlb_active_r ? sys_invtlb_va_bit12_r : ds_tlb_s1_va_bit12_r;
assign tlb_s1_asid_to_tlb     = sys_invtlb_active_r ? sys_invtlb_asid_r     : ds_tlb_s1_asid_r;
assign tlb_invtlb_op_to_tlb   = sys_invtlb_active_r ? sys_invtlb_op_r       : invtlb_op;

// TLB write/invalidate side effects are committed only from sys_stage registers.
// This cuts the path ds_inst/fs_to_ds_bus_r -> TLB search/exception cone -> TLB CE.
assign tlb_local_exc  = exc_int | exc_adef | exc_ine | exc_tlbr_i | exc_pif | exc_ppi_i;
assign tlb_side_ready = sys_commit_fire && !sys_exc_taken_r;
assign tlb_we      = tlb_side_ready && (sys_inst_tlbwr_r | sys_inst_tlbfill_r);
assign tlb_w_index = sys_inst_tlbfill_r ? sys_tlbfill_index_r : csr_tlbidx[4:0];
assign tlb_w_e     = ~csr_tlbidx[31];
assign tlb_w_vppn  = csr_tlbehi[31:13];
assign tlb_w_ps    = csr_tlbidx[29:24];
assign tlb_w_asid  = csr_asid[9:0];
assign tlb_w_g     = csr_tlbelo0[6] & csr_tlbelo1[6];

assign tlb_w_ppn0  = csr_tlbelo0[27:8];
assign tlb_w_plv0  = csr_tlbelo0[3:2];
assign tlb_w_mat0  = csr_tlbelo0[5:4];
assign tlb_w_d0    = csr_tlbelo0[1];
assign tlb_w_v0    = csr_tlbelo0[0];

assign tlb_w_ppn1  = csr_tlbelo1[27:8];
assign tlb_w_plv1  = csr_tlbelo1[3:2];
assign tlb_w_mat1  = csr_tlbelo1[5:4];
assign tlb_w_d1    = csr_tlbelo1[1];
assign tlb_w_v1    = csr_tlbelo1[0];

assign tlb_invtlb_valid = tlb_side_ready && sys_inst_invtlb_r;

tlb #(.TLBNUM(32)) u_tlb(
    .clk          (clk),

    .s0_vppn      (ds_tlb_s0_vppn_r),
    .s0_va_bit12  (ds_tlb_s0_va_bit12_r),
    .s0_asid      (ds_tlb_s0_asid_r),
    .s0_found     (tlb_s0_found),
    .s0_index     (tlb_s0_index),
    .s0_ppn       (tlb_s0_ppn),
    .s0_ps        (tlb_s0_ps),
    .s0_plv       (tlb_s0_plv),
    .s0_mat       (tlb_s0_mat),
    .s0_d         (tlb_s0_d),
    .s0_v         (tlb_s0_v),

    .s1_vppn      (tlb_s1_vppn_to_tlb),
    .s1_va_bit12  (tlb_s1_va_bit12_to_tlb),
    .s1_asid      (tlb_s1_asid_to_tlb),
    .s1_found     (tlb_s1_found),
    .s1_index     (tlb_s1_index),
    .s1_ppn       (tlb_s1_ppn),
    .s1_ps        (tlb_s1_ps),
    .s1_plv       (tlb_s1_plv),
    .s1_mat       (tlb_s1_mat),
    .s1_d         (tlb_s1_d),
    .s1_v         (tlb_s1_v),

    .invtlb_valid (tlb_invtlb_valid),
    .invtlb_op    (tlb_invtlb_op_to_tlb),

    .we           (tlb_we),
    .w_index      (tlb_w_index),
    .w_e          (tlb_w_e),
    .w_vppn       (tlb_w_vppn),
    .w_ps         (tlb_w_ps),
    .w_asid       (tlb_w_asid),
    .w_g          (tlb_w_g),
    .w_ppn0       (tlb_w_ppn0),
    .w_plv0       (tlb_w_plv0),
    .w_mat0       (tlb_w_mat0),
    .w_d0         (tlb_w_d0),
    .w_v0         (tlb_w_v0),
    .w_ppn1       (tlb_w_ppn1),
    .w_plv1       (tlb_w_plv1),
    .w_mat1       (tlb_w_mat1),
    .w_d1         (tlb_w_d1),
    .w_v1         (tlb_w_v1),

    .r_index      (csr_tlbidx[4:0]),
    .r_e          (tlb_r_e),
    .r_vppn       (tlb_r_vppn),
    .r_ps         (tlb_r_ps),
    .r_asid       (tlb_r_asid),
    .r_g          (tlb_r_g),
    .r_ppn0       (tlb_r_ppn0),
    .r_plv0       (tlb_r_plv0),
    .r_mat0       (tlb_r_mat0),
    .r_d0         (tlb_r_d0),
    .r_v0         (tlb_r_v0),
    .r_ppn1       (tlb_r_ppn1),
    .r_plv1       (tlb_r_plv1),
    .r_mat1       (tlb_r_mat1),
    .r_d1         (tlb_r_d1),
    .r_v1         (tlb_r_v1)
);

wire unused_tlb_search0;
wire unused_tlb_search1;
assign unused_tlb_search0 = tlb_s0_found | (|tlb_s0_index) | (|tlb_s0_ppn) | (|tlb_s0_ps) |
                            (|tlb_s0_plv) | (|tlb_s0_mat) | tlb_s0_d | tlb_s0_v;
assign unused_tlb_search1 = (|tlb_s1_ppn) | (|tlb_s1_ps) | (|tlb_s1_plv) |
                            (|tlb_s1_mat) | tlb_s1_d | tlb_s1_v;

// ---------------- exp19: DMW/TLB virtual-to-physical translation ----------------
wire        crmd_da;
wire        crmd_pg;
wire [ 1:0] crmd_plv;
wire        direct_addr_mode;
wire        page_mode;

assign crmd_plv = csr_crmd[1:0];
assign crmd_da  = csr_crmd[3];
assign crmd_pg  = csr_crmd[4];
assign direct_addr_mode = crmd_da & ~crmd_pg;
assign page_mode        = ~crmd_da & crmd_pg;

wire dmw0_plv_hit;
wire dmw1_plv_hit;
wire inst_dmw0_hit;
wire inst_dmw1_hit;
wire data_dmw0_hit;
wire data_dmw1_hit;
wire fast_dmw_boundary_safe;
wire fast_data_dmw0_hit;
wire fast_data_dmw1_hit;
wire fast_data_dmw_hit;
wire inst_dmw_hit;
wire data_dmw_hit;
wire [31:0] inst_dmw_paddr;
wire [31:0] data_dmw_paddr;
wire [31:0] data_tlb_paddr;
wire [31:0] mem_addr_phy_id;

// LoongArch MAT encoding used by CRMD.DATM, DMWn.MAT and TLB entries.
// 00: strongly-ordered uncached (SUC)
// 01: coherent cached (CC)
// 10: weakly-ordered uncached (WUC)
// 11: reserved; treat as uncached (fail closed) in this implementation.
localparam [1:0] MAT_SUC = 2'b00;
localparam [1:0] MAT_CC  = 2'b01;
localparam [1:0] MAT_WUC = 2'b10;

wire [1:0] data_dmw_mat;
wire [1:0] data_tlb_mat;
wire [1:0] data_mat_id;
wire        data_uncached_id;
wire        inst_need_tlb;
wire        data_need_tlb;
wire        tlb_s0_plv_fail;
wire        tlb_s1_plv_fail;
wire        exc_tlbr_d;
wire        exc_tlbr;
wire        exc_pil;
wire        exc_pis;
wire        exc_ppi_d;
wire        exc_ppi;
wire        exc_pme;
wire        exc_tlb_related;
wire [31:0] exc_tlb_badv;
wire        ds_tlb_read_like;
wire        ds_tlb_access_like;
wire        ds_pre_exc_tlbr_i;
wire        ds_pre_exc_tlbr_d;
wire        ds_pre_exc_tlbr;
wire        ds_pre_exc_pif;
wire        ds_pre_exc_pil;
wire        ds_pre_exc_pis;
wire        ds_pre_exc_ppi_i;
wire        ds_pre_exc_ppi_d;
wire        ds_pre_exc_ppi;
wire        ds_pre_exc_pme;
wire [31:0] ds_pre_data_tlb_paddr;
wire [31:0] ds_pre_mem_addr_phy;
wire [31:0] ds_pre_exc_tlb_badv;
wire        ds_lookup_tlb_s0_plv_fail;
wire        ds_lookup_tlb_s1_plv_fail;
wire        ds_lookup_exc_tlbr_i;
wire        ds_lookup_exc_tlbr_d;
wire        ds_lookup_exc_tlbr;
wire        ds_lookup_exc_pif;
wire        ds_lookup_exc_pil;
wire        ds_lookup_exc_pis;
wire        ds_lookup_exc_ppi_i;
wire        ds_lookup_exc_ppi_d;
wire        ds_lookup_exc_ppi;
wire        ds_lookup_exc_pme;
wire [31:0] ds_lookup_data_tlb_paddr;
wire [31:0] ds_lookup_mem_addr_phy;
wire [31:0] ds_lookup_exc_tlb_badv;

assign dmw0_plv_hit = ((crmd_plv == 2'd0) && csr_dmw0[0]) || ((crmd_plv == 2'd3) && csr_dmw0[3]);
assign dmw1_plv_hit = ((crmd_plv == 2'd0) && csr_dmw1[0]) || ((crmd_plv == 2'd3) && csr_dmw1[3]);

assign inst_dmw0_hit = page_mode && dmw0_plv_hit && (ds_pc[31:29] == csr_dmw0[31:29]);
assign inst_dmw1_hit = page_mode && dmw1_plv_hit && (ds_pc[31:29] == csr_dmw1[31:29]);
assign data_dmw0_hit = ds_addr_mmu_active && page_mode && dmw0_plv_hit && (mem_addr_id[31:29] == csr_dmw0[31:29]);
assign data_dmw1_hit = ds_addr_mmu_active && page_mode && dmw1_plv_hit && (mem_addr_id[31:29] == csr_dmw1[31:29]);
// M18.5T timing cut for the zero-bubble DMW path.  A signed 12-bit memory
// offset cannot change a 512MB virtual segment unless the base is in the first
// or last 4KB of that segment.  Classify all other accesses from rj[31:29]
// directly, removing the 32-bit effective-address carry chain from ID ready
// and the IF-queue write enables.  Boundary-near accesses conservatively use
// the original registered address precheck below, which preserves exact
// cross-segment behavior and exception reporting.
assign fast_dmw_boundary_safe = (|mem_base_value[28:12]) &&
                                !(&mem_base_value[28:12]);
assign fast_data_dmw0_hit = ds_addr_mmu_active && fast_dmw_boundary_safe &&
                            page_mode && dmw0_plv_hit &&
                            (mem_base_value[31:29] == csr_dmw0[31:29]);
assign fast_data_dmw1_hit = ds_addr_mmu_active && fast_dmw_boundary_safe &&
                            page_mode && dmw1_plv_hit &&
                            (mem_base_value[31:29] == csr_dmw1[31:29]);
assign fast_data_dmw_hit = fast_data_dmw0_hit | fast_data_dmw1_hit;
assign inst_dmw_hit  = inst_dmw0_hit | inst_dmw1_hit;
assign data_dmw_hit  = data_dmw0_hit | data_dmw1_hit;
assign inst_dmw_paddr = inst_dmw0_hit ? {csr_dmw0[27:25], ds_pc[28:0]} :
                        inst_dmw1_hit ? {csr_dmw1[27:25], ds_pc[28:0]} : ds_pc;
assign data_dmw_paddr = data_dmw0_hit ? {csr_dmw0[27:25], mem_addr_id[28:0]} :
                        data_dmw1_hit ? {csr_dmw1[27:25], mem_addr_id[28:0]} : mem_addr_id;

assign inst_need_tlb = page_mode && !inst_dmw_hit;
assign data_need_tlb = ds_addr_mmu_active && page_mode && !data_dmw_hit;

assign data_tlb_paddr = (tlb_s1_ps == 6'd22) ? {tlb_s1_ppn[19:10], mem_addr_id[21:0]} :
                                                {tlb_s1_ppn,        mem_addr_id[11:0]};
assign mem_addr_phy_id = direct_addr_mode ? mem_addr_id :
                         data_dmw_hit     ? data_dmw_paddr :
                         data_need_tlb    ? data_tlb_paddr :
                                            mem_addr_id;

// Select MAT from the same translation path that selected the physical address.
// For a TLB-mapped access, the one-cycle ID precheck has already latched the
// translation result before the instruction can enter EXE; use the latched MAT
// in that case so PA and MAT always belong to the same TLB response.
assign data_dmw_mat = data_dmw0_hit ? csr_dmw0[5:4] :
                      data_dmw1_hit ? csr_dmw1[5:4] : MAT_CC;
assign data_tlb_mat = ds_pre_tlb_s1_mat_r;
assign data_mat_id  = direct_addr_mode ? csr_crmd[6:5] :
                      data_dmw_hit     ? data_dmw_mat :
                      data_need_tlb    ? data_tlb_mat :
                                         MAT_CC;

// Only coherent-cached accesses enter DCache.  SUC/WUC (and reserved MAT=11)
// use the uncached AXI path, preventing stale cache data or write-back traffic
// from reaching MMIO devices.
assign data_uncached_id = (data_mat_id != MAT_CC);

assign tlb_s0_plv_fail = (crmd_plv > tlb_s0_plv);
assign tlb_s1_plv_fail = (crmd_plv > tlb_s1_plv);

assign exc_tlbr_i = ds_valid && inst_need_tlb && !exc_adef && !tlb_s0_found;
assign exc_tlbr_d = ds_valid && data_need_tlb && !exc_ale && !tlb_s1_found;
assign exc_tlbr   = exc_tlbr_i | exc_tlbr_d;

assign ds_tlb_read_like   = ds_mem_load | inst_cacop;
assign ds_tlb_access_like = ds_mem_access | inst_cacop;

assign exc_pif = ds_valid && inst_need_tlb && !exc_adef && tlb_s0_found && !tlb_s0_v;
assign exc_pil = ds_valid && data_need_tlb && ds_tlb_read_like && !exc_ale && tlb_s1_found && !tlb_s1_v;
assign exc_pis = ds_valid && data_need_tlb && ds_mem_store && !exc_ale && tlb_s1_found && !tlb_s1_v;
assign exc_ppi_i = ds_valid && inst_need_tlb && !exc_adef && tlb_s0_found && tlb_s0_v && tlb_s0_plv_fail;
assign exc_ppi_d = ds_valid && data_need_tlb && ds_tlb_access_like && !exc_ale && tlb_s1_found && tlb_s1_v && tlb_s1_plv_fail;
assign exc_ppi = exc_ppi_i | exc_ppi_d;
assign exc_pme = ds_valid && data_need_tlb && ds_mem_store && !exc_ale &&
                 tlb_s1_found && tlb_s1_v && !tlb_s1_plv_fail && !tlb_s1_d;

assign exc_tlb_related = exc_tlbr | exc_pif | exc_pil | exc_pis | exc_ppi | exc_pme;
assign exc_tlb_badv = (exc_tlbr_i | exc_pif | exc_ppi_i) ? ds_pc : mem_addr_id;

assign ds_pre_exc_tlbr_i = ds_pre_inst_need_tlb_r && !ds_pre_exc_adef_r && !ds_pre_tlb_s0_found_r;
assign ds_pre_exc_tlbr_d = ds_pre_data_need_tlb_r && !ds_pre_exc_ale_r && !ds_pre_tlb_s1_found_r;
assign ds_pre_exc_tlbr   = ds_pre_exc_tlbr_i | ds_pre_exc_tlbr_d;
assign ds_pre_exc_pif = ds_pre_inst_need_tlb_r && !ds_pre_exc_adef_r &&
                        ds_pre_tlb_s0_found_r && !ds_pre_tlb_s0_v_r;
assign ds_pre_exc_pil = ds_pre_data_need_tlb_r && ds_pre_tlb_read_like_r && !ds_pre_exc_ale_r &&
                        ds_pre_tlb_s1_found_r && !ds_pre_tlb_s1_v_r;
assign ds_pre_exc_pis = ds_pre_data_need_tlb_r && ds_pre_mem_store_r && !ds_pre_exc_ale_r &&
                        ds_pre_tlb_s1_found_r && !ds_pre_tlb_s1_v_r;
assign ds_pre_exc_ppi_i = ds_pre_inst_need_tlb_r && !ds_pre_exc_adef_r &&
                          ds_pre_tlb_s0_found_r && ds_pre_tlb_s0_v_r && ds_pre_tlb_s0_plv_fail_r;
assign ds_pre_exc_ppi_d = ds_pre_data_need_tlb_r && ds_pre_tlb_access_like_r && !ds_pre_exc_ale_r &&
                          ds_pre_tlb_s1_found_r && ds_pre_tlb_s1_v_r && ds_pre_tlb_s1_plv_fail_r;
assign ds_pre_exc_ppi = ds_pre_exc_ppi_i | ds_pre_exc_ppi_d;
assign ds_pre_exc_pme = ds_pre_data_need_tlb_r && ds_pre_mem_store_r && !ds_pre_exc_ale_r &&
                        ds_pre_tlb_s1_found_r && ds_pre_tlb_s1_v_r &&
                        !ds_pre_tlb_s1_plv_fail_r && !ds_pre_tlb_s1_d_r;
assign ds_pre_data_tlb_paddr = (ds_pre_tlb_s1_ps_r == 6'd22) ?
                               {ds_pre_tlb_s1_ppn_r[19:10], ds_pre_mem_addr_id_r[21:0]} :
                               {ds_pre_tlb_s1_ppn_r,        ds_pre_mem_addr_id_r[11:0]};
assign ds_pre_mem_addr_phy = ds_pre_direct_addr_mode_r ? ds_pre_mem_addr_id_r :
                             ds_pre_data_dmw_hit_r     ? ds_pre_data_dmw_paddr_r :
                             ds_pre_data_need_tlb_r    ? ds_pre_data_tlb_paddr :
                                                          ds_pre_mem_addr_id_r;
assign ds_pre_exc_tlb_badv = (ds_pre_exc_tlbr_i | ds_pre_exc_pif | ds_pre_exc_ppi_i) ?
                             ds_pre_pc_r : ds_pre_mem_addr_id_r;

assign ds_lookup_tlb_s0_plv_fail = (ds_pre_crmd_plv_r > tlb_s0_plv);
assign ds_lookup_tlb_s1_plv_fail = (ds_pre_crmd_plv_r > tlb_s1_plv);
assign ds_lookup_exc_tlbr_i = ds_pre_inst_need_tlb_r && !ds_pre_exc_adef_r && !tlb_s0_found;
assign ds_lookup_exc_tlbr_d = ds_pre_data_need_tlb_r && !ds_pre_exc_ale_r && !tlb_s1_found;
assign ds_lookup_exc_tlbr   = ds_lookup_exc_tlbr_i | ds_lookup_exc_tlbr_d;
assign ds_lookup_exc_pif = ds_pre_inst_need_tlb_r && !ds_pre_exc_adef_r &&
                           tlb_s0_found && !tlb_s0_v;
assign ds_lookup_exc_pil = ds_pre_data_need_tlb_r && ds_pre_tlb_read_like_r && !ds_pre_exc_ale_r &&
                           tlb_s1_found && !tlb_s1_v;
assign ds_lookup_exc_pis = ds_pre_data_need_tlb_r && ds_pre_mem_store_r && !ds_pre_exc_ale_r &&
                           tlb_s1_found && !tlb_s1_v;
assign ds_lookup_exc_ppi_i = ds_pre_inst_need_tlb_r && !ds_pre_exc_adef_r &&
                             tlb_s0_found && tlb_s0_v && ds_lookup_tlb_s0_plv_fail;
assign ds_lookup_exc_ppi_d = ds_pre_data_need_tlb_r && ds_pre_tlb_access_like_r && !ds_pre_exc_ale_r &&
                             tlb_s1_found && tlb_s1_v && ds_lookup_tlb_s1_plv_fail;
assign ds_lookup_exc_ppi = ds_lookup_exc_ppi_i | ds_lookup_exc_ppi_d;
assign ds_lookup_exc_pme = ds_pre_data_need_tlb_r && ds_pre_mem_store_r && !ds_pre_exc_ale_r &&
                           tlb_s1_found && tlb_s1_v &&
                           !ds_lookup_tlb_s1_plv_fail && !tlb_s1_d;
assign ds_lookup_data_tlb_paddr = (tlb_s1_ps == 6'd22) ?
                                  {tlb_s1_ppn[19:10], ds_pre_mem_addr_id_r[21:0]} :
                                  {tlb_s1_ppn,        ds_pre_mem_addr_id_r[11:0]};
assign ds_lookup_mem_addr_phy = ds_pre_direct_addr_mode_r ? ds_pre_mem_addr_id_r :
                                ds_pre_data_dmw_hit_r     ? ds_pre_data_dmw_paddr_r :
                                ds_pre_data_need_tlb_r    ? ds_lookup_data_tlb_paddr :
                                                             ds_pre_mem_addr_id_r;
assign ds_lookup_exc_tlb_badv = (ds_lookup_exc_tlbr_i | ds_lookup_exc_pif | ds_lookup_exc_ppi_i) ?
                                ds_pre_pc_r : ds_pre_mem_addr_id_r;

// Instruction TLB checks and data/cacheop addresses use the precheck registers.
// This keeps translated data addresses off the wide DS->ES bus combinational path.
// TLB maintenance instructions are still serialized by sys_stage_valid and sample
// the current TLB search result in their normal capture cycle; otherwise TLBSRCH
// can commit a stale miss into CSR_TLBIDX.
assign ds_addr_precheck_needed = ds_addr_mmu_active | inst_tlbsrch;
// Slow path: a real TLB lookup/search requires the registered request/result
// protocol.  Fast path: direct-address or DMW data accesses only need their
// already-computed physical address captured once; they do not need the extra
// TLB classification cycle.
assign ds_tlb_check_needed = ds_valid && (inst_need_tlb | data_need_tlb | inst_tlbsrch);
`ifdef DISABLE_M185_FAST_DMW
assign ds_fast_addr_check_needed = ds_valid && ds_addr_mmu_active &&
                                   ((!direct_addr_mode &&
                                     !inst_need_tlb && !data_need_tlb) ||
                                    (direct_addr_mode && exc_ale_now));
`else
assign ds_fast_addr_check_needed = ds_valid && ds_addr_mmu_active &&
                                   (((direct_addr_mode || fast_data_dmw_hit) &&
                                     exc_ale_now) ||
                                    (!direct_addr_mode &&
                                     !fast_data_dmw_hit &&
                                     !inst_need_tlb && !data_need_tlb));
`endif
// Round-6 direct-address bypass:
// A DA=1,PG=0 load/store/CACOP needs neither TLB classification nor DMW
// translation.  Let it enter EXE immediately and use the existing EXE ALU
// result as its physical address.  Paging-mode DMW and real TLB accesses keep
// the registered precheck protocol.
`ifdef DISABLE_M185_FAST_DMW
assign ds_precheck_needed = ds_valid &&
                            (inst_need_tlb | inst_tlbsrch |
                             (ds_addr_mmu_active && !direct_addr_mode) |
                             (direct_addr_mode && exc_ale_now));
`else
assign ds_precheck_needed = ds_valid &&
                            (inst_need_tlb | inst_tlbsrch |
                             (ds_addr_mmu_active &&
                              !direct_addr_mode && !fast_data_dmw_hit) |
                             ((direct_addr_mode || fast_data_dmw_hit) &&
                              exc_ale_now));
`endif
assign exc_tlb_taken_pre = exc_tlb_related;
assign exc_tlb_ecode_pre = exc_tlbr ? 6'h3f :
                           exc_pil  ? 6'h01 :
                           exc_pis  ? 6'h02 :
                           exc_pif  ? 6'h03 :
                           exc_pme  ? 6'h04 :
                           exc_ppi  ? 6'h07 : 6'h00;
assign exc_tlb_badv_pre = exc_tlb_badv;

assign exc_tlb_taken_r = ds_tlb_exc_tlbr_r | ds_tlb_exc_pif_r | ds_tlb_exc_pil_r |
                         ds_tlb_exc_pis_r  | ds_tlb_exc_pme_r | ds_tlb_exc_ppi_r;
assign exc_tlb_ecode_r = ds_tlb_exc_tlbr_r ? 6'h3f :
                         ds_tlb_exc_pil_r  ? 6'h01 :
                         ds_tlb_exc_pis_r  ? 6'h02 :
                         ds_tlb_exc_pif_r  ? 6'h03 :
                         ds_tlb_exc_pme_r  ? 6'h04 :
                         ds_tlb_exc_ppi_r  ? 6'h07 : 6'h00;

// Round 12 timing cut:
// Every instruction/data TLB access already waits for the registered lookup
// protocol to assert ds_tlb_checked_r.  Direct-address and DMW accesses cannot
// raise a TLB exception.  Therefore the architectural exception selector only
// needs the registered result; it must not re-evaluate the current effective
// address, DMW hit and combinational TLB search on the EXE-valid path.
// Fast DMW address capture also sets ds_tlb_checked_r, but it does not perform
// a TLB lookup; ds_tlb_exc_* may still contain the previous lookup's payload on
// that edge.  Qualify the registered result with the registered request class so
// only an instruction/data access that actually needed the TLB can consume it.
assign ds_registered_tlb_access = ds_pre_inst_need_tlb_r | ds_pre_data_need_tlb_r;
assign exc_tlb_taken_sel   = ds_tlb_checked_r && ds_registered_tlb_access && exc_tlb_taken_r;
assign exc_tlb_ecode_sel   = exc_tlb_ecode_r;
assign exc_tlb_badv_sel    = ds_tlb_exc_badv_r;
assign exc_tlbr_sel        = ds_tlb_checked_r && ds_registered_tlb_access && ds_tlb_exc_tlbr_r;
assign exc_tlb_related_sel = ds_tlb_checked_r && ds_registered_tlb_access && exc_tlb_taken_r;

wire [31:0] rj_value_to_es;
wire [31:0] imm_to_es;
// Direct-address accesses do not need an ID/MMU precheck cycle.  Their address
// is the normal rj+imm ALU result in EXE.  DMW/TLB accesses still use the
// registered translated physical address.
assign direct_addr_exe_bypass = direct_addr_mode && (ds_mem_access | inst_cacop);
`ifdef DISABLE_M185_FAST_DMW
assign dmw_addr_exe_bypass = 1'b0;
assign dmw_pseg_to_es      = 3'b0;
`else
assign dmw_addr_exe_bypass = fast_data_dmw_hit && (ds_mem_access | inst_cacop);
assign dmw_pseg_to_es = fast_data_dmw0_hit ? csr_dmw0[27:25] : csr_dmw1[27:25];
`endif
// A DMW-only access has already captured its translated address in the precheck
// payload register.  Use that register directly; reserve ds_mem_addr_phy_r for
// real TLB lookup results.  This removes the current 32-bit address adder/DMW
// mux from the ds_mem_addr_phy_r D path.
assign mem_addr_phy_to_es = (ds_mem_access | inst_cacop) ?
                            (ds_pre_data_dmw_hit_r ? ds_pre_data_dmw_paddr_r :
                                                     ds_mem_addr_phy_r) : 32'b0;
assign rj_value_to_es = rj_value;
assign imm_to_es      = imm;


assign src_no_rd = ~(inst_st_b | inst_st_h | inst_st_w | inst_sc_w)
                & ~(inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu)
                & ~(inst_csrwr | inst_csrxchg)
                & ~(inst_tlbsrch | inst_tlbrd | inst_tlbwr | inst_tlbfill | inst_invtlb)
                & ~(inst_rdcntid_w | inst_rdcntvl_w | inst_rdcntvh_w | inst_brk);

// Slot-1 candidate operand forwarding.  Current MEM non-load results are
// already registered and remain safe to consume in ID.  Round 13 additionally
// records matches against the current main/side EXE instructions and performs
// those forwards one cycle later, from the corresponding MEM registers, inside
// the side EXE lane.  A main-lane load is still blocked: forwarding cache/AXI
// load data through the side ALU would create an unsafe long path.
assign slot1_rj_value = ((slot1_rj != 5'b0) && (slot1_rj == slot1_ms_dest)) ? slot1_ms_result :
                        ((slot1_rj != 5'b0) && (slot1_rj == ms_to_ds_dest) && !ms_to_ds_load_op) ? ms_to_ds_result :
                        slot1_rf_rdata1;
assign slot1_rk_value = ((slot1_src2_reg != 5'b0) && (slot1_src2_reg == slot1_ms_dest)) ? slot1_ms_result :
                        ((slot1_src2_reg != 5'b0) && (slot1_src2_reg == ms_to_ds_dest) && !ms_to_ds_load_op) ? ms_to_ds_result :
                        slot1_rf_rdata2;

// M18.2 younger-branch operand network.  Branches consume their operands in
// ID, so unlike a side-lane ALU they cannot defer an EXE dependency into the
// next cycle.  MEM non-load values are safe to forward; an EXE producer or a
// MEM load makes the pair fall back to ordinary single issue.
assign slot1_branch_rj_es_match = slot1_branch_cond && (slot1_rj != 5'b0) &&
                                  (slot1_rj == es_to_ds_dest);
assign slot1_branch_rd_es_match = slot1_branch_cond && (slot1_rd != 5'b0) &&
                                  (slot1_rd == es_to_ds_dest);
assign slot1_branch_rj_side_es_match = slot1_branch_cond && (slot1_rj != 5'b0) &&
                                       (slot1_rj == slot1_es_dest);
assign slot1_branch_rd_side_es_match = slot1_branch_cond && (slot1_rd != 5'b0) &&
                                       (slot1_rd == slot1_es_dest);
assign slot1_branch_rj_ms_match = slot1_branch_cond && (slot1_rj != 5'b0) &&
                                  (slot1_rj == ms_to_ds_dest);
assign slot1_branch_rd_ms_match = slot1_branch_cond && (slot1_rd != 5'b0) &&
                                  (slot1_rd == ms_to_ds_dest);
assign slot1_branch_rj_side_ms_match = slot1_branch_cond && (slot1_rj != 5'b0) &&
                                       (slot1_rj == slot1_ms_dest);
assign slot1_branch_rd_side_ms_match = slot1_branch_cond && (slot1_rd != 5'b0) &&
                                       (slot1_rd == slot1_ms_dest);
assign slot1_branch_pipeline_hazard =
       slot1_branch_rj_es_match | slot1_branch_rd_es_match |
       slot1_branch_rj_side_es_match | slot1_branch_rd_side_es_match |
       (ms_to_ds_load_op &&
        (slot1_branch_rj_ms_match | slot1_branch_rd_ms_match));
// Pair admission is already guarded by the exact slot0_dual_predecode
// whitelist.  Every instruction in that whitelist writes rd, so the later
// gr_we/dest decode is redundant here and only drags exception/ERTN logic into
// the FIFO pop-count cone.
assign slot1_branch_pair_raw = (rd != 5'b0) && slot1_branch_cond &&
                               ((slot1_rj == rd) || (slot1_rd == rd));

assign slot1_branch_rj_value = slot1_branch_rj_side_ms_match ? slot1_ms_result :
                               (slot1_branch_rj_ms_match && !ms_to_ds_load_op) ?
                               ms_to_ds_result : slot1_rf_rdata1;
assign slot1_branch_rd_value = slot1_branch_rd_side_ms_match ? slot1_ms_result :
                               (slot1_branch_rd_ms_match && !ms_to_ds_load_op) ?
                               ms_to_ds_result : slot1_rf_rdata2;

wire slot1_branch_eq = (slot1_branch_rj_value == slot1_branch_rd_value);
wire slot1_branch_lt_signed =
     ($signed(slot1_branch_rj_value) < $signed(slot1_branch_rd_value));
wire slot1_branch_lt_unsigned =
     (slot1_branch_rj_value < slot1_branch_rd_value);
assign slot1_branch_taken_raw = slot1_branch_direct ||
       ((slot1_candidate_inst[31:26] == 6'h16) &&  slot1_branch_eq) ||
       ((slot1_candidate_inst[31:26] == 6'h17) && !slot1_branch_eq) ||
       ((slot1_candidate_inst[31:26] == 6'h18) &&  slot1_branch_lt_signed) ||
       ((slot1_candidate_inst[31:26] == 6'h19) && !slot1_branch_lt_signed) ||
       ((slot1_candidate_inst[31:26] == 6'h1a) &&  slot1_branch_lt_unsigned) ||
       ((slot1_candidate_inst[31:26] == 6'h1b) && !slot1_branch_lt_unsigned);
assign slot1_branch_offset = slot1_branch_direct ?
       {{4{slot1_candidate_inst[9]}}, slot1_candidate_inst[9:0],
        slot1_candidate_inst[25:10], 2'b0} :
       {{14{slot1_candidate_inst[25]}}, slot1_candidate_inst[25:10], 2'b0};
assign slot1_branch_target = slot1_candidate_pc + slot1_branch_offset;
assign slot1_branch_fallthrough = slot1_candidate_pc + 32'd4;
assign slot1_branch_direction_miss =
       slot1_candidate_pred_taken ^ slot1_branch_taken_raw;
assign slot1_branch_target_miss = slot1_candidate_pred_taken &&
                                  slot1_branch_taken_raw &&
                                  (slot1_candidate_pred_nextpc != slot1_branch_target);
assign slot1_branch_mispredict = pair_branch_fire &&
                                 (slot1_branch_direction_miss ||
                                  slot1_branch_target_miss);

assign slot1_src1_main_es_match = slot1_src1_used && (slot1_rj != 5'b0) &&
                                  (slot1_rj == es_to_ds_dest);
assign slot1_src2_main_es_match = slot1_src2_used && (slot1_src2_reg != 5'b0) &&
                                  (slot1_src2_reg == es_to_ds_dest);
assign slot1_src1_side_es_match = slot1_src1_used && (slot1_rj != 5'b0) &&
                                  (slot1_rj == slot1_es_dest);
assign slot1_src2_side_es_match = slot1_src2_used && (slot1_src2_reg != 5'b0) &&
                                  (slot1_src2_reg == slot1_es_dest);

// Side-EXE producers are one-cycle ALU operations and are always registered in
// side MEM before the new Slot-1 instruction consumes them.  Main-EXE producers
// are likewise forwardable once they are able to leave EXE, except loads.
assign slot1_src1_es_hazard = slot1_src1_main_es_match && es_to_ds_load_op;
assign slot1_src2_es_hazard = slot1_src2_main_es_match && es_to_ds_load_op;
// Round 14 released-load forwarding for Slot 1:
// When a matching main-lane load is leaving MEM on this edge, its architectural
// value is registered in main WB before the side-lane ALU consumes it next cycle.
// Carry a one-bit selector into side EXE and forward only from the WB register.
// A load that is not leaving MEM remains a hard issue hazard, so no cache/AXI
// data is introduced into the ID pair-admission datapath.
assign slot1_src1_ms_load_match = slot1_src1_used && ms_to_ds_load_op &&
                                  (slot1_rj != 5'b0) && (slot1_rj == ms_to_ds_dest);
assign slot1_src2_ms_load_match = slot1_src2_used && ms_to_ds_load_op &&
                                  (slot1_src2_reg != 5'b0) && (slot1_src2_reg == ms_to_ds_dest);
`ifdef DISABLE_SLOT1_LOAD_RELEASE
assign slot1_src1_ms_load_hazard = slot1_src1_ms_load_match;
assign slot1_src2_ms_load_hazard = slot1_src2_ms_load_match;
`else
assign slot1_src1_ms_load_hazard = slot1_src1_ms_load_match && !ms_to_ds_load_leave;
assign slot1_src2_ms_load_hazard = slot1_src2_ms_load_match && !ms_to_ds_load_leave;
`endif

// Intra-pair dependencies. WAR is safe because both operands are captured
// before either instruction writes back. RAW/WAW matter only when Slot 0 really
// writes a GPR; this allows stores and other non-GPR instructions in Slot 0.
assign slot1_pair_raw = (rd != 5'b0) &&
                        ((slot1_src1_used && (slot1_rj == rd)) ||
                         (slot1_src2_used && (slot1_src2_reg == rd)));
assign slot1_pair_waw = (rd != 5'b0) &&
                        (slot1_rd != 5'b0) && (slot1_rd == rd);

assign slot1_imm = (slot1_imm_sel == 2'b11) ? {slot1_i20, 12'b0} :
                   (slot1_imm_sel == 2'b10) ? {27'b0, slot1_rk} :
                   (slot1_imm_sel == 2'b01) ? {20'b0, slot1_i12} :
                                                     {{20{slot1_i12[11]}}, slot1_i12};

assign current_simple_alu = inst_add_w | inst_sub_w | inst_slt | inst_sltu |
                            inst_nor | inst_and | inst_or | inst_xor |
                            inst_slli_w | inst_srli_w | inst_srai_w |
                            inst_addi_w | inst_slti | inst_sltui |
                            inst_andi | inst_ori | inst_xori |
                            inst_sll_w | inst_srl_w | inst_sra_w |
                            inst_lu12i_w | inst_pcaddu12i;

// Normal orientation: old instruction remains in the main lane and the young
// simple ALU instruction enters the side lane.  Swapped orientation: the old
// simple ALU instruction enters the side lane while the young ordinary memory
// instruction uses the single main LSU.  The swapped path is deliberately
// restricted to aligned direct-address accesses, so it does not duplicate TLB,
// CSR or exception machinery.
// M17.0T timing recovery: when reverse memory swap is disabled, remove the
// WB-forward -> address-adder -> DMW/CSR compare -> issue-enable cone at
// preprocessing time.  This is stronger than relying on post-synthesis
// constant propagation and keeps normal dual issue untouched.
`ifdef DISABLE_M16_LANE_SWAP
assign swap_candidate_es_load_hazard = 1'b0;
assign swap_candidate_side_es_hazard = 1'b0;
assign swap_candidate_ms_load_hazard = 1'b0;
assign swap_candidate_base = 32'b0;
assign swap_candidate_store_data = 32'b0;
assign swap_candidate_addr = 32'b0;
assign swap_candidate_aligned = 1'b0;
assign swap_candidate_dmw0_hit = 1'b0;
assign swap_candidate_dmw1_hit = 1'b0;
assign swap_candidate_dmw_hit = 1'b0;
assign swap_candidate_dmw_paddr = 32'b0;
assign swap_candidate_dmw_mat = MAT_CC;
assign swap_candidate_addrmode_ok = 1'b0;
assign swap_candidate_direct_bypass = 1'b0;
assign swap_candidate_uncached = 1'b0;
assign swap_candidate_phy_addr = 32'b0;
assign swap_pair_raw = 1'b0;
assign swap_pair_waw = 1'b0;
assign swap_rj_from_next_ms = 1'b0;
assign swap_rd_from_next_ms = 1'b0;
`else
assign swap_candidate_es_load_hazard =
       (slot1_src1_main_es_match || slot1_src2_main_es_match) && es_to_ds_load_op;
assign swap_candidate_side_es_hazard = slot1_src1_side_es_match || slot1_src2_side_es_match;
assign swap_candidate_ms_load_hazard = ms_to_ds_load_op &&
       (((slot1_rj != 5'b0) && (slot1_rj == ms_to_ds_dest)) ||
        ((slot1_src2_used && slot1_src2_reg != 5'b0) && (slot1_src2_reg == ms_to_ds_dest)));

assign swap_candidate_base = (slot1_src1_main_es_match && !es_to_ds_load_op) ? es_to_ds_result :
                             slot1_rj_value;
assign swap_candidate_store_data = (slot1_src2_main_es_match && !es_to_ds_load_op) ? es_to_ds_result :
                                   slot1_rk_value;
assign swap_candidate_addr = swap_candidate_base + {{20{slot1_i12[11]}}, slot1_i12};
assign swap_candidate_aligned =
       (slot1_predecoded_mem_op[0] || slot1_predecoded_mem_op[3] || slot1_predecoded_mem_op[5]) ? 1'b1 :
       (slot1_predecoded_mem_op[1] || slot1_predecoded_mem_op[4] || slot1_predecoded_mem_op[6]) ? ~swap_candidate_addr[0] :
       (slot1_predecoded_mem_op[2] || slot1_predecoded_mem_op[7]) ? (swap_candidate_addr[1:0] == 2'b00) : 1'b0;

// M16.1R: performance programs execute in paging mode through a DMW, so the
// previous direct-address-only rule made the reverse ALU+memory orientation
// dead code.  A DMW hit needs no TLB lookup and has no page permission fault.
// Admit only coherent-cached DMW mappings; uncached/MMIO DMW accesses remain
// serialized to avoid introducing younger side effects before the older ALU.
assign swap_candidate_dmw0_hit = page_mode && dmw0_plv_hit &&
                                 (swap_candidate_addr[31:29] == csr_dmw0[31:29]);
assign swap_candidate_dmw1_hit = page_mode && dmw1_plv_hit &&
                                 (swap_candidate_addr[31:29] == csr_dmw1[31:29]);
assign swap_candidate_dmw_hit = swap_candidate_dmw0_hit | swap_candidate_dmw1_hit;
assign swap_candidate_dmw_paddr = swap_candidate_dmw0_hit ?
                                  {csr_dmw0[27:25], swap_candidate_addr[28:0]} :
                                  {csr_dmw1[27:25], swap_candidate_addr[28:0]};
assign swap_candidate_dmw_mat = swap_candidate_dmw0_hit ? csr_dmw0[5:4] :
                                swap_candidate_dmw1_hit ? csr_dmw1[5:4] : MAT_CC;
assign swap_candidate_addrmode_ok = direct_addr_mode ||
                                    (swap_candidate_dmw_hit &&
                                     (swap_candidate_dmw_mat == MAT_CC));
assign swap_candidate_direct_bypass = direct_addr_mode;
assign swap_candidate_uncached = direct_addr_mode && (csr_crmd[6:5] != MAT_CC);
assign swap_candidate_phy_addr = swap_candidate_dmw_hit ?
                                 swap_candidate_dmw_paddr : 32'b0;
assign swap_pair_raw = (rd != 5'b0) &&
                       (((slot1_rj != 5'b0) && (slot1_rj == rd)) ||
                        (slot1_candidate_store && (slot1_src2_reg != 5'b0) &&
                         (slot1_src2_reg == rd)));
assign swap_pair_waw = (rd != 5'b0) && slot1_candidate_load &&
                       (slot1_rd != 5'b0) && (slot1_rd == rd);
assign swap_rj_from_next_ms = slot1_src1_main_es_match && !es_to_ds_load_op;
assign swap_rd_from_next_ms = slot1_candidate_store && slot1_src2_main_es_match && !es_to_ds_load_op;

`endif

// M16.2M: reverse ALU+MUL pairing.  The older simple ALU uses the side lane
// and the younger multiply uses the existing main-lane DSP path.  No second
// multiplier is instantiated.  Main-EXE results may be consumed one cycle
// later from MEM; side-EXE dependencies remain serialized because the main
// lane has no side-MEM operand selector.
`ifdef DISABLE_M162_ALU_MUL_SWAP
assign mul_swap_src1_main_es_match = 1'b0;
assign mul_swap_src2_main_es_match = 1'b0;
assign mul_swap_src1_side_es_match = 1'b0;
assign mul_swap_src2_side_es_match = 1'b0;
assign mul_swap_src1_ms_load_match = 1'b0;
assign mul_swap_src2_ms_load_match = 1'b0;
assign mul_swap_es_load_hazard = 1'b0;
assign mul_swap_side_es_hazard = 1'b0;
assign mul_swap_ms_load_hazard = 1'b0;
assign mul_swap_pair_raw = 1'b0;
assign mul_swap_pair_waw = 1'b0;
assign mul_swap_rj_from_next_ms = 1'b0;
assign mul_swap_rk_from_next_ms = 1'b0;
assign mul_swap_rj_from_next_ws = 1'b0;
assign mul_swap_rk_from_next_ws = 1'b0;
`else
assign mul_swap_src1_main_es_match = (slot1_rj != 5'b0) &&
                                     (slot1_rj == es_to_ds_dest);
assign mul_swap_src2_main_es_match = (slot1_rk != 5'b0) &&
                                     (slot1_rk == es_to_ds_dest);
assign mul_swap_src1_side_es_match = (slot1_rj != 5'b0) &&
                                     (slot1_rj == slot1_es_dest);
assign mul_swap_src2_side_es_match = (slot1_rk != 5'b0) &&
                                     (slot1_rk == slot1_es_dest);
assign mul_swap_src1_ms_load_match = ms_to_ds_load_op &&
                                     (slot1_rj != 5'b0) &&
                                     (slot1_rj == ms_to_ds_dest);
assign mul_swap_src2_ms_load_match = ms_to_ds_load_op &&
                                     (slot1_rk != 5'b0) &&
                                     (slot1_rk == ms_to_ds_dest);
assign mul_swap_es_load_hazard =
       (mul_swap_src1_main_es_match || mul_swap_src2_main_es_match) &&
       es_to_ds_load_op;
assign mul_swap_side_es_hazard = mul_swap_src1_side_es_match ||
                                 mul_swap_src2_side_es_match;
assign mul_swap_ms_load_hazard =
       (mul_swap_src1_ms_load_match || mul_swap_src2_ms_load_match) &&
       !ms_to_ds_load_leave;
assign mul_swap_pair_raw = (rd != 5'b0) &&
                           (((slot1_rj != 5'b0) && (slot1_rj == rd)) ||
                            ((slot1_rk != 5'b0) && (slot1_rk == rd)));
assign mul_swap_pair_waw = (rd != 5'b0) &&
                           (slot1_rd != 5'b0) && (slot1_rd == rd);
assign mul_swap_rj_from_next_ms = mul_swap_src1_main_es_match &&
                                   !es_to_ds_load_op;
assign mul_swap_rk_from_next_ms = mul_swap_src2_main_es_match &&
                                   !es_to_ds_load_op;
assign mul_swap_rj_from_next_ws = mul_swap_src1_ms_load_match &&
                                   ms_to_ds_load_leave;
assign mul_swap_rk_from_next_ws = mul_swap_src2_ms_load_match &&
                                   ms_to_ds_load_leave;
`endif

assign slot1_issue_alu_op = pair_branch_fire ? 12'b0 :
                            pair_swap_fire ? alu_op : slot1_predecoded_alu_op;
assign slot1_issue_src1 = pair_branch_fire ? 32'b0 :
                          pair_swap_fire ? (src1_is_pc ? ds_pc : rj_value) :
                          (slot1_src1_is_pc ? slot1_candidate_pc :
                           slot1_src1_used   ? slot1_rj_value : 32'b0);
assign slot1_issue_src2 = pair_branch_fire ? 32'b0 :
                          pair_swap_fire ? (src2_is_imm ? imm : rkd_value) :
                          (slot1_src2_used ? slot1_rk_value : slot1_imm);
assign slot1_issue_dest = pair_branch_fire ? 5'b0 :
                          pair_swap_fire ? rd : slot1_rd;
assign slot1_issue_pc   = pair_swap_fire ? ds_pc : slot1_candidate_pc;
assign slot1_issue_inst = pair_swap_fire ? ds_inst : slot1_candidate_inst;
assign slot1_issue_is_older = pair_swap_fire;

assign slot1_issue_src1_from_main_ms = pair_branch_fire ? 1'b0 :
                                       pair_swap_fire ? rj_from_next_ms : slot1_src1_main_es_match;
assign slot1_issue_src2_from_main_ms = pair_branch_fire ? 1'b0 :
                                       pair_swap_fire ? rk_from_next_ms : slot1_src2_main_es_match;
`ifdef DISABLE_SLOT1_LOAD_RELEASE
assign slot1_issue_src1_from_main_ws = pair_branch_fire ? 1'b0 :
                                       pair_swap_fire ? rj_from_next_ws : 1'b0;
assign slot1_issue_src2_from_main_ws = pair_branch_fire ? 1'b0 :
                                       pair_swap_fire ? rk_from_next_ws : 1'b0;
`else
assign slot1_issue_src1_from_main_ws = pair_branch_fire ? 1'b0 :
                                       pair_swap_fire ? rj_from_next_ws :
                                       (!slot1_src1_main_es_match &&
                                        !slot1_src1_side_es_match &&
                                        slot1_src1_ms_load_match && ms_to_ds_load_leave);
assign slot1_issue_src2_from_main_ws = pair_branch_fire ? 1'b0 :
                                       pair_swap_fire ? rk_from_next_ws :
                                       (!slot1_src2_main_es_match &&
                                        !slot1_src2_side_es_match &&
                                        slot1_src2_ms_load_match && ms_to_ds_load_leave);
`endif
assign slot1_issue_src1_from_side_ms = (pair_branch_fire || pair_swap_fire) ?
                                       1'b0 : slot1_src1_side_es_match;
assign slot1_issue_src2_from_side_ms = (pair_branch_fire || pair_swap_fire) ?
                                       1'b0 : slot1_src2_side_es_match;

assign rj_wait = ~src_no_rj && (rj != 5'b0) &&
                 ((rj == es_to_ds_dest) || (rj == ms_to_ds_dest) || (rj == ws_to_ds_dest));
assign rk_wait = ~src_no_rk && (rk != 5'b0) &&
                 ((rk == es_to_ds_dest) || (rk == ms_to_ds_dest) || (rk == ws_to_ds_dest));
assign rd_wait = ~src_no_rd && (rd != 5'b0) &&
                 ((rd == es_to_ds_dest) || (rd == ms_to_ds_dest) || (rd == ws_to_ds_dest));

assign no_wait = ~rj_wait & ~rk_wait & ~rd_wait;

// Keep equality out of a long device-wide carry comparator.  Four local
// byte reductions plus one LUT-level AND are both exact and substantially
// easier to place beside the distributed branch-read copies.
wire [31:0] br_rj_xor_rd = br_rj_value ^ br_rd_value;
(* keep = "true" *) wire br_eq_byte0 = ~(|br_rj_xor_rd[ 7: 0]);
(* keep = "true" *) wire br_eq_byte1 = ~(|br_rj_xor_rd[15: 8]);
(* keep = "true" *) wire br_eq_byte2 = ~(|br_rj_xor_rd[23:16]);
(* keep = "true" *) wire br_eq_byte3 = ~(|br_rj_xor_rd[31:24]);
assign rj_eq_rd = br_eq_byte0 & br_eq_byte1 & br_eq_byte2 & br_eq_byte3;
wire rj_lt_rd_signed;
wire rj_lt_rd_unsigned;
assign rj_lt_rd_signed   = ($signed(br_rj_value) < $signed(br_rd_value));
assign rj_lt_rd_unsigned = (br_rj_value < br_rd_value);
assign br_taken_raw = (   inst_beq  &&  rj_eq_rd
                       || inst_bne  && !rj_eq_rd
                       || inst_blt  &&  rj_lt_rd_signed
                       || inst_bge  && !rj_lt_rd_signed
                       || inst_bltu &&  rj_lt_rd_unsigned
                       || inst_bgeu && !rj_lt_rd_unsigned
                       || inst_bl
                       || inst_b
                    );
assign mem_addr_id   = mem_base_value + imm;
// Alignment depends only on the low two effective-address bits.  Computing it
// with a 2-bit adder avoids placing the complete 32-bit carry chain on the
// exception/EXE-valid path.  Page-mode and DMW accesses use the value captured
// during their mandatory precheck.  A rare misaligned direct-address access is
// also staged for one cycle below so BADV can use the registered full address;
// aligned direct-address accesses retain the zero-precheck fast path.
assign mem_addr_low2 = mem_base_value[1:0] + imm[1:0];
assign exc_int  = ((csr_estat[12:0] & csr_ecfg[12:0]) != 13'b0) && csr_crmd[2] && !inst_ertn;
assign exc_adef = (ds_pc[1:0] != 2'b00);
assign exc_ale_now = ds_mem_mmu_active &&
                     (((inst_ld_h | inst_ld_hu | inst_st_h) & mem_addr_low2[0]) |
                      ((inst_ld_w | inst_st_w | inst_ll_w | inst_sc_w) & (mem_addr_low2 != 2'b00)));
assign exc_ale = ds_tlb_checked_r ? ds_pre_exc_ale_r : exc_ale_now;
assign exc_sys  = inst_syscall;
assign exc_brk  = inst_brk;
assign exc_ine  = ~inst_valid & ~exc_adef;
assign exc_taken = exc_int | exc_adef | exc_ale | exc_tlb_taken_sel |
                   exc_sys | exc_brk | exc_ine;
    // IDLE can only be woken by an interrupt or a fetch-side exception.
    // Use the instruction-only TLB qualification below rather than the generic
    // exc_tlb_taken_sel.  The generic selector also contains effective-address
    // addition, DMW classification and data-TLB exceptions; even though IDLE is
    // mutually exclusive with a data access, that cone survived synthesis and
    // formed the routed Round-8 worst path through idle_wait -> ds_allowin ->
    // the 97-bit IF/ID payload register bank.
    assign idle_exception_pending = exc_int | exc_adef |
                                    (inst_need_tlb &&
                                     (ds_tlb_exc_tlbr_r | ds_tlb_exc_pif_r |
                                      ds_tlb_exc_ppi_r));
    assign idle_wait = inst_idle && !idle_exception_pending;

assign exc_ecode = exc_int  ? 6'h00 :
                   exc_adef ? 6'h08 :
                   exc_ale  ? 6'h09 :
                   exc_tlb_taken_sel ? exc_tlb_ecode_sel :
                   exc_sys  ? 6'h0b :
                   exc_brk  ? 6'h0c :
                   exc_ine  ? 6'h0d : 6'h00;
assign exc_esubcode = 9'b0;
assign exc_badv = (exc_adef) ? ds_pc :
                  // ALE is committed only after the one-cycle address capture,
                  // so the exact full bad address comes from the registered
                  // payload rather than the current 32-bit adder.
                  (exc_ale ) ? ds_pre_mem_addr_id_r :
                  (exc_tlb_taken_sel) ? exc_tlb_badv_sel : 32'b0;

// Stage the operations whose side effects previously closed timing inside ID:
// exceptions/ERTN/TLB maintenance and CSR writes.  CSRRD/RDCNT remain single-cycle
// because they only read values and do not update the CSR/TLB state.
assign sys_stage_inst = exc_taken | inst_ertn | csr_we | inst_tlbsrch | inst_tlbrd |
                        inst_tlbwr | inst_tlbfill | inst_invtlb_valid_op;
assign sys_taken = sys_stage_valid ? (sys_exc_taken_r | sys_inst_ertn_r) :
                                     (exc_taken | inst_ertn);
assign sys_taken_commit = sys_commit_fire && (sys_exc_taken_r | sys_inst_ertn_r);
assign sys_br_target = sys_inst_ertn_r ? csr_era :
                       sys_exc_tlbr_r ? {csr_tlbrentry[31:6], 6'b0} :
                                        {csr_eentry[31:6], 6'b0};

// Branch decision can use forwarded operands, so it no longer waits for all RAW hazards.
// But load-use data is not available in EXE, so load_stall must also block br_taken.
// CSR/exception/ertn instructions are serialized until older stages are empty, which keeps
// side effects precise enough for this simple in-order pipeline.
assign older_pipe_empty  = ~es_stage_valid & ~ms_stage_valid & ~ws_stage_valid &
                           ~slot1_es_stage_valid & ~slot1_ms_stage_valid & ~slot1_ws_stage_valid;
assign id_simple_ready_go = ds_valid & ~es_raw_stall & ~slot1_es_raw_stall &
                            ~load_stall & ~ms_load_stall &
                            ~mem_addr_es_stall & ~branch_src_stall;
// exp19 ref-local v7:
// Do not feed current ID TLB/exception decode back into ds_allowin/fs_to_ds_bus_r CE.
// A system/exception/TLB instruction is captured into sys_* registers as soon as
// its source operands are stable, then the pipeline is held by the registered
// sys_stage_valid until older stages are empty and the side effect is committed.
// This follows the senior design's "decode now, commit later" idea without
// transplanting its whole WBU/CSR framework.
assign sys_stall = (!sys_stage_valid && (sys_src_es_stall || idle_wait));
assign ds_tlb_precheck_stall = ds_precheck_needed && !ds_tlb_checked_r;
assign ds_tlb_lookup_fire = ds_valid && id_simple_ready_go && ~sys_stage_valid && ~sys_stall &&
                            ds_tlb_check_needed && !ds_tlb_lookup_r && !ds_tlb_checked_r;
assign ds_fast_addr_capture_fire = ds_valid && id_simple_ready_go && ~sys_stage_valid && ~sys_stall &&
                                   ds_fast_addr_check_needed && !ds_tlb_lookup_r && !ds_tlb_checked_r;
assign ds_tlb_classify_fire = ds_tlb_lookup_r && !ds_tlb_checked_r;
assign normal_ready_go = id_simple_ready_go & ~sys_stall &
                         ~ds_tlb_precheck_stall;

assign sys_capture_fire = ds_valid && normal_ready_go && !sys_stage_valid;
assign sys_stage_start  = sys_capture_fire && sys_stage_inst;
assign sys_commit_fire  = sys_stage_valid && older_pipe_empty;
assign normal_commit_fire = ds_valid && !sys_stage_valid && ds_ready_go && es_allowin;

// Phase 3 widens Slot 0 eligibility. Slot 1 is still a simple integer ALU lane,
// but Slot 0 may now be a load/store, LL/SC, multiply/divide, counter read,
// CPUCFG, CSRRD, or ordinary integer operation. Control transfers, CACOP, IDLE,
// exceptions and side-effecting CSR/TLB/system instructions remain serialized.
// Shared EXE/MEM/WB backpressure keeps a paired Slot 1 instruction aligned even
// when Slot 0 has variable latency. Page-crossing pairs are still split because
// only Slot 0 has completed the instruction-side translation/permission check.
`ifdef DISABLE_ROUND15_SLOT0_PREDECODE
assign slot0_dual_eligible = !ctrl_transfer_inst && !sys_stage_inst &&
                             !inst_cacop && !inst_idle;
`else
assign slot0_dual_eligible = ds_slot0_dual_static_ok;
`endif

// ds_to_es_valid already enforces exceptions, ERTN, registered system-stage
// serialization and external flush.  Keep pair legality limited to static
// instruction class, adjacency and true inter-instruction hazards.
assign slot1_pair_legal = fs_to_ds_valid && slot0_dual_eligible && slot1_candidate_simple &&
                          (slot1_candidate_pc == (ds_pc + 32'd4)) &&
                          (slot1_candidate_pc[31:12] == ds_pc[31:12]) &&
                          !slot1_src1_es_hazard && !slot1_src2_es_hazard &&
                          !slot1_src1_ms_load_hazard && !slot1_src2_ms_load_hazard &&
                          !slot1_pair_raw && !slot1_pair_waw;

`ifdef DISABLE_M16_LANE_SWAP
assign pair_swap_mem_legal = 1'b0;
`else
assign pair_swap_mem_legal = fs_to_ds_valid && current_simple_alu && slot1_candidate_mem &&
                             swap_candidate_addrmode_ok &&
                             (slot1_candidate_pc == (ds_pc + 32'd4)) &&
                             (slot1_candidate_pc[31:12] == ds_pc[31:12]) &&
                             !swap_candidate_es_load_hazard &&
                             !swap_candidate_side_es_hazard &&
                             !swap_candidate_ms_load_hazard &&
                             !swap_pair_raw && !swap_pair_waw && swap_candidate_aligned;
`endif
`ifdef DISABLE_M162_ALU_MUL_SWAP
assign pair_swap_mul_legal = 1'b0;
`else
assign pair_swap_mul_legal = fs_to_ds_valid && current_simple_alu && slot1_candidate_mul &&
                             (slot1_candidate_pc == (ds_pc + 32'd4)) &&
                             (slot1_candidate_pc[31:12] == ds_pc[31:12]) &&
                             !mul_swap_es_load_hazard &&
                             !mul_swap_side_es_hazard &&
                             !mul_swap_ms_load_hazard &&
                             !mul_swap_pair_raw && !mul_swap_pair_waw;
`endif
assign pair_swap_legal = pair_swap_mem_legal | pair_swap_mul_legal;
`ifdef DISABLE_M182_SLOT1_BRANCH_PAIR
assign pair_branch_legal = 1'b0;
`else
assign pair_branch_legal = fs_to_ds_valid && slot0_dual_eligible &&
                           (slot1_branch_cond || slot1_branch_direct) &&
                           (slot1_candidate_pc == (ds_pc + 32'd4)) &&
                           (slot1_candidate_pc[31:12] == ds_pc[31:12]) &&
                           !slot1_branch_pipeline_hazard &&
                           !slot1_branch_pair_raw;
`endif
// M19 timing cut:
// slot0_dual_eligible is now an exact whitelist of ordinary architectural
// instructions.  For that whitelist, syscall/break/INE/ERTN/system-stage
// qualification in ds_to_es_valid is redundant.  Keeping those decoders in the
// pair-fire expression created a selector -> full exception decode -> selector
// feedback path.  Retain every dynamic condition that can still prevent an
// ordinary instruction from issuing: dependencies, address/TLB precheck,
// asynchronous interrupt, architectural address exceptions, backend readiness
// and external flush.
assign pair_interrupt_pending = ((csr_estat[12:0] & csr_ecfg[12:0]) != 13'b0) &&
                                csr_crmd[2];
// The fast pair whitelist contains no memory/LLSC operation, so ALE and the
// data-address/TLB cone are architecturally impossible on this path.
assign pair_exception_pending = pair_interrupt_pending | exc_adef |
                                exc_tlb_taken_sel;
assign pair_issue_ready = id_simple_ready_go && !sys_stage_valid &&
                          !ds_tlb_precheck_stall && es_allowin &&
                          !external_flush && !pair_exception_pending;

assign pair_normal_fire   = pair_issue_ready && slot1_pair_legal;
assign pair_swap_mem_fire = pair_issue_ready && !slot1_pair_legal &&
                            pair_swap_mem_legal;
assign pair_swap_mul_fire = pair_issue_ready && !slot1_pair_legal &&
                            !pair_swap_mem_legal && pair_swap_mul_legal;
assign pair_swap_fire = pair_swap_mem_fire | pair_swap_mul_fire;
assign pair_branch_fire = pair_issue_ready &&
                          !slot1_pair_legal && !pair_swap_legal &&
                          pair_branch_legal;
assign dual_pair_fire = pair_normal_fire | pair_swap_fire | pair_branch_fire;
assign slot1_issue_valid = dual_pair_fire;

// M19.8 timing cut:
// The IF FIFO pop count and the one-bit refill-bank selector are consumed only
// on an ID allow-in edge.  For a valid resident instruction, that edge already
// proves id_simple_ready_go, !sys_stage_valid, !ds_tlb_precheck_stall and
// es_allowin.  Repeating those ready terms through dual_pair_fire created the
// routed data_ok -> MEM -> EXE -> ID -> FIFO/selector path.  Precompute only the
// pairing decision that remains architecturally relevant on the allow-in edge.
// Actual Slot-1 issue continues to use dual_pair_fire, so this changes neither
// issue qualification nor the instruction stream.
assign dual_pair_preselect = slot1_pair_legal | pair_swap_mem_legal |
                             pair_swap_mul_legal | pair_branch_legal;
assign pair_refill_select = ds_valid && !sys_stage_valid &&
                            !external_flush && !pair_exception_pending &&
                            dual_pair_preselect;

// The current ID instruction was already removed from the IF FIFO when loaded.
// pop_base consumes FIFO head as the ordinary next Slot 0.  A legal pair uses
// that head as Slot 1 and pop_extra consumes head+1 for the next Slot-0 refill.
assign fs_pop_base  = ds_allowin && fs_to_ds_valid;
assign fs_pop_extra = pair_refill_select && fs_to_ds_valid1;

// exp22 v3 timing fix:
// A current-ID exception/TLB decode must not feed the IF-stage fs_valid/nextpc
// control path.  Exceptions/ERTN/TLB/CSR side effects have already been captured
// into sys_* registers and are redirected only by sys_taken_commit.
//
// With the dynamic predictor, the normal branch path redirects IF only on a
// misprediction.  This retains the existing safety rule: the instruction must
// be accepted by EXE before it can flush IF.  Correct predictions do not
// create a flush.  Exceptions/ERTN/CACOP always retain priority over normal
// branch recovery.
assign ctrl_transfer_inst = inst_beq  || inst_bne  || inst_blt  || inst_bge ||
                            inst_bltu || inst_bgeu || inst_bl   || inst_b   ||
                            inst_jirl;
assign br_actual_target = ds_pc + br_offs;
assign br_fallthrough_pc = ds_pc + 32'd4;
// Split direction and target misses.  The target compare is meaningful only
// when both prediction and architectural outcome are taken, which shortens the
// common not-taken branch cone and avoids comparing an unused fall-through PC.
wire bp_direction_miss = ds_pred_taken ^ br_taken_raw;
// The BTB is fully tagged and is invalidated with CACOP, while each
// PC-relative target is immutable between such flushes.  Its target therefore
// cannot disagree after a hit; only direction participates in the ID redirect
// cone.  Dynamic jirl targets retain an explicit target comparison in EXE.
wire bp_target_miss    = 1'b0;
// Keep the data-address adder, DMW classifier and data-TLB exception cone out
// of branch recovery/training.  For a valid branch/jirl those exception classes
// are architecturally impossible; instruction-side exceptions remain fully
// checked, using the registered TLB result when a lookup was required.
assign bp_inst_tlb_exception = ds_tlb_checked_r && ds_pre_inst_need_tlb_r &&
                               (ds_tlb_exc_tlbr_r | ds_tlb_exc_pif_r |
                                ds_tlb_exc_ppi_r);
assign bp_update_exception = exc_int | exc_adef | bp_inst_tlb_exception;

wire id_early_ctrl_inst = ctrl_transfer_inst &&
                          !deferred_load_branch_candidate;
assign bp_mispredict = normal_commit_fire && id_early_ctrl_inst &&
                       !bp_update_exception && !inst_ertn && !external_flush &&
                       bp_direction_miss;
assign id_br_stall              = br_stall;
assign id_normal_redirect_valid = bp_mispredict | slot1_branch_mispredict;
assign id_normal_actual_taken   = pair_branch_fire ? slot1_branch_taken_raw :
                                                       br_taken_raw;
assign id_normal_taken_target   = pair_branch_fire ? slot1_branch_target :
                                                       br_actual_target;
assign id_normal_fallthrough    = pair_branch_fire ? slot1_branch_fallthrough :
                                                       br_fallthrough_pc;
assign id_sys_redirect_valid    = sys_taken_commit;
assign id_sys_redirect_target   = sys_br_target;

// Train only from an architecturally executed control transfer.
// - Conditional branches update their 2-bit BHT counter and their BTB target.
// - b/bl update a direct BTB entry.
// - jirl records a last target; standard ABI returns additionally train RAS.
wire slot0_bp_update_en = normal_commit_fire && id_early_ctrl_inst &&
                          !bp_update_exception && !inst_ertn && !external_flush;
wire slot0_bp_update_is_return = inst_jirl && (rd == 5'd0) &&
                                  (rj == 5'd1) && (i16 == 16'b0);
assign bp_update_en        = pair_branch_fire | slot0_bp_update_en;
assign bp_update_pc        = pair_branch_fire ? slot1_candidate_pc : ds_pc;
assign bp_update_is_cond   = pair_branch_fire ? slot1_branch_cond :
                             (inst_beq || inst_bne || inst_blt || inst_bge ||
                              inst_bltu || inst_bgeu);
assign bp_update_taken     = pair_branch_fire ? slot1_branch_taken_raw :
                                                br_taken_raw;
assign bp_update_target    = pair_branch_fire ? slot1_branch_target :
                                                br_actual_target;
assign bp_update_is_call   = pair_branch_fire ? 1'b0 :
                             (inst_bl || (inst_jirl && (rd == 5'd1)));
assign bp_update_is_return = pair_branch_fire ? 1'b0 :
                             slot0_bp_update_is_return;
assign bp_update_is_indirect = pair_branch_fire ? 1'b0 :
                               (inst_jirl && !slot0_bp_update_is_return);

// If a branch/jirl depends on a load in EXE, prevent IF from using an unfinished target.
assign load_stall = es_to_ds_load_op &&
                    (((rj == es_to_ds_dest) && rj_wait) ||
                     ((rk == es_to_ds_dest) && rk_wait) ||
                     ((rd == es_to_ds_dest) && rd_wait)) &&
                    !deferred_load_branch_candidate;
// MEM-stage load data is intentionally not forwarded into ID, because doing
// so recreates the long cache/AXI -> ID/MMU/branch path.  The consumer may,
// however, enter EXE on the edge that moves the load into WB.  WB->EXE forwarding
// then supplies the value in the consumer's first EXE cycle.
//
// Round 8 required EXE to be empty because a transient WB value could disappear
// if the consumer was subsequently held in EXE.  Round 9 adds an EXE operand-hold
// register bank: on the first blocked EXE cycle, the fully forwarded operands are
// captured and remain stable until the instruction leaves.  This makes late-WB
// release safe even when an intervening instruction moves into MEM and blocks.
//
// Address-generating dependencies are still conservative: branch/jirl and
// CSR/TLB/system instructions consume operands in ID, while a load/store/CACOP
// base dependency would make the ID address precheck use an old value.  A store
// whose base is independent but whose write data depends on the load is safe,
// because the write data is consumed only in EXE and is covered by operand hold.
assign ms_load_rj_match = (!src_no_rj) && (rj != 5'b0) && (rj == ms_to_ds_dest);
assign ms_load_rk_match = (!src_no_rk) && (rk != 5'b0) && (rk == ms_to_ds_dest);
assign ms_load_rd_match = (!src_no_rd) && (rd != 5'b0) && (rd == ms_to_ds_dest);
assign ms_load_src_match = ms_load_rj_match | ms_load_rk_match | ms_load_rd_match;

assign ms_load_mem_addr_dep = (ds_mem_access | ds_cacheop_access) &&
                              ms_load_rj_match;

assign ms_load_release_safe = ms_to_ds_load_leave &&
                              !sys_inst_no_exc &&
                              !inst_cpucfg &&
                              !ms_load_mem_addr_dep &&
                              (!ds_cacheop_access) &&
                              (!ds_mem_access || ds_mem_store);

assign ms_load_stall = ms_to_ds_load_op && ms_load_src_match &&
                       !ms_load_release_safe;

// Predecode the only values that must arrive after the ID->EXE edge.
// - A matching ordinary EXE producer advances to MEM on the acceptance edge.
// - A matching released MEM load advances to WB on that same edge.
// An EXE match has priority over an older MEM match to preserve in-order RAW
// semantics when two older instructions write the same architectural register.
assign rj_from_next_ms = rj_es_match && !es_to_ds_load_op;
assign rk_from_next_ms = rk_es_match && !es_to_ds_load_op;
assign rd_from_next_ms = rd_es_match && !es_to_ds_load_op;
assign rj_from_next_ws = !rj_es_match && ms_to_ds_load_op &&
                         ms_load_release_safe && ms_load_rj_match;
assign rk_from_next_ws = !rk_es_match && ms_to_ds_load_op &&
                         ms_load_release_safe && ms_load_rk_match;
assign rd_from_next_ws = !rd_es_match && ms_to_ds_load_op &&
                         ms_load_release_safe && ms_load_rd_match;

assign br_stall   = (es_raw_stall | slot1_es_raw_stall | load_stall | ms_load_stall | mem_addr_es_stall |
                     branch_src_stall | sys_stall | ds_tlb_precheck_stall |
                     (sys_stage_valid && !sys_commit_fire)) && ds_valid;


// Performance-counter classification.  Count only when ID holds a valid
// instruction; this avoids reset/bubble cycles polluting the stall numbers.
assign perf_load_stall_o         = ds_valid && load_stall;
assign perf_ms_load_stall_o      = ds_valid && ms_load_stall;
assign perf_branch_src_stall_o   = ds_valid && branch_src_stall;
assign perf_es_raw_stall_o       = ds_valid && es_raw_stall;
assign perf_mem_addr_es_stall_o  = ds_valid && mem_addr_es_stall;
assign perf_sys_stall_o          = ds_valid && sys_stall;
assign perf_tlb_precheck_stall_o = ds_valid && ds_tlb_precheck_stall;
assign perf_ds_blocked_o         = ds_valid && !ds_allowin;
assign perf_dual_issue_o         = dual_pair_fire;


// In the current soc_bram func environment the extra ext_int port is usually
// not driven by the testbench.  If it is sampled directly, ESTAT.IS[9:2]
// becomes X and csrrd ESTAT mismatches the golden trace.
// Default to no external hardware interrupt.  If your top/testbench really
// drives ext_int, define USE_EXT_INT or replace this with: assign hw_int = ext_int.
`ifdef USE_EXT_INT
assign hw_int = ext_int;
`else
assign hw_int = 8'b0;
`endif

assign csr_num = ds_inst[23:10];
assign csr_tval  = timer_cnt;
// LLSC_STAGE62A_DONE
assign csr_ticlr = 32'b0;
assign llbit_state = csr_llbctl[0];

assign rdcnt_result = inst_rdcntvl_w ? stable_counter[31:0]  :
                      inst_rdcntvh_w ? stable_counter[63:32] :
                      csr_tid;

assign cpucfg_result = (rj_value == 32'h0)  ? 32'h0000_0000 :
                       (rj_value == 32'h1)  ? 32'h0000_0000 :
                       (rj_value == 32'h2)  ? 32'h0000_0000 :
                       (rj_value == 32'h10) ? 32'h0000_0000 :
                                              32'h0000_0000;

assign csr_rvalue = inst_cpucfg ? cpucfg_result :
                    (inst_rdcntid_w | inst_rdcntvl_w | inst_rdcntvh_w) ? rdcnt_result :
                    (csr_num == CSR_CRMD  ) ? csr_crmd   :
                    (csr_num == CSR_PRMD  ) ? csr_prmd   :
                    (csr_num == CSR_ECFG  ) ? csr_ecfg   :
                    (csr_num == CSR_ESTAT ) ? csr_estat  :
                    (csr_num == CSR_ERA   ) ? csr_era    :
                    (csr_num == CSR_BADV  ) ? csr_badv   :
                    (csr_num == CSR_EENTRY) ? csr_eentry :
                    (csr_num == CSR_SAVE0 ) ? csr_save0  :
                    (csr_num == CSR_SAVE1 ) ? csr_save1  :
                    (csr_num == CSR_SAVE2 ) ? csr_save2  :
                    (csr_num == CSR_SAVE3 ) ? csr_save3  :
                    (csr_num == CSR_TLBIDX) ? csr_tlbidx :
                    (csr_num == CSR_TLBEHI) ? csr_tlbehi :
                    (csr_num == CSR_TLBELO0) ? csr_tlbelo0 :
                    (csr_num == CSR_TLBELO1) ? csr_tlbelo1 :
                    (csr_num == CSR_ASID  ) ? csr_asid   :
                    (csr_num == CSR_TLBRENTRY) ? csr_tlbrentry :
                    (csr_num == CSR_DMW0  ) ? csr_dmw0   :
                    (csr_num == CSR_DMW1  ) ? csr_dmw1   :
                    (csr_num == CSR_TID   ) ? csr_tid    :
                    (csr_num == CSR_LLBCTL) ? csr_llbctl :
                    (csr_num == CSR_TCFG  ) ? csr_tcfg   :
                    (csr_num == CSR_TVAL  ) ? csr_tval   :
                    (csr_num == CSR_TICLR ) ? csr_ticlr  : 32'b0;

assign csr_we     = inst_csrwr | inst_csrxchg;
assign csr_wmask  = inst_csrxchg ? rj_value : 32'hffff_ffff;
assign csr_wvalue = rkd_value;
assign csr_wdata  = (csr_rvalue & ~csr_wmask) | (csr_wvalue & csr_wmask);
assign res_from_csr = inst_csrrd | inst_csrwr | inst_csrxchg |
                      inst_rdcntid_w | inst_rdcntvl_w | inst_rdcntvh_w | inst_cpucfg;
assign timer_enabled    = csr_tcfg[0];
assign timer_periodic   = csr_tcfg[1];
assign timer_init_value = {csr_tcfg[31:2], 2'b0};

wire csr_do_update;
assign csr_do_update = sys_commit_fire;

assign csr_we_commit    = sys_csr_we_r;
assign csr_num_commit   = sys_csr_num_r;
assign csr_wdata_commit = sys_csr_wdata_r;

// exp19 ref-local v8:
// CSR/TLB side effects are committed only when sys_commit_fire is true.
// Therefore the CSR update cone must use only registered sys_* values.
// Do NOT keep a "sys_stage_valid ? sys_* : current_ID" fallback here, otherwise
// Vivado still sees a false-but-timed path from fs_to_ds_bus_r/current TLB search
// into csr_crmd/csr_tcfg/csr_tlbehi D/R pins.
assign csr_exc_taken       = sys_exc_taken_r;
assign csr_exc_ecode       = sys_exc_ecode_r;
assign csr_exc_esubcode    = sys_exc_esubcode_r;
assign csr_exc_badv        = sys_exc_badv_r;
assign csr_exc_tlbr        = sys_exc_tlbr_r;
assign csr_exc_adef        = sys_exc_adef_r;
assign csr_exc_ale         = sys_exc_ale_r;
assign csr_exc_tlb_related = sys_exc_tlb_related_r;
assign csr_pc              = sys_pc_r;
assign csr_inst_ertn       = sys_inst_ertn_r;
assign csr_inst_tlbsrch    = sys_inst_tlbsrch_r;
assign csr_inst_tlbrd      = sys_inst_tlbrd_r;
assign csr_inst_tlbfill    = sys_inst_tlbfill_r;
assign csr_tlbsrch_found   = sys_tlb_s1_found_r;
assign csr_tlbsrch_index   = sys_tlb_s1_index_r;
assign csr_tlbrd_e         = sys_tlb_r_e_r;
assign csr_tlbrd_vppn      = sys_tlb_r_vppn_r;
assign csr_tlbrd_ps        = sys_tlb_r_ps_r;
assign csr_tlbrd_asid      = sys_tlb_r_asid_r;
assign csr_tlbrd_g         = sys_tlb_r_g_r;
assign csr_tlbrd_ppn0      = sys_tlb_r_ppn0_r;
assign csr_tlbrd_plv0      = sys_tlb_r_plv0_r;
assign csr_tlbrd_mat0      = sys_tlb_r_mat0_r;
assign csr_tlbrd_d0        = sys_tlb_r_d0_r;
assign csr_tlbrd_v0        = sys_tlb_r_v0_r;
assign csr_tlbrd_ppn1      = sys_tlb_r_ppn1_r;
assign csr_tlbrd_plv1      = sys_tlb_r_plv1_r;
assign csr_tlbrd_mat1      = sys_tlb_r_mat1_r;
assign csr_tlbrd_d1        = sys_tlb_r_d1_r;
assign csr_tlbrd_v1        = sys_tlb_r_v1_r;

// exp19 opt19: isolate ESTAT update from the long CSR/TLB exception cone.
// The previous coding style let Vivado map exception-code clears onto FDRE R pins,
// so the path fs_to_ds_bus_r -> data TLB/MMU -> exc_ecode -> csr_estat[R]
// became the routed critical path.  Build a normal next-state value and drive
// csr_estat through D instead; reset remains the only real reset path.
wire        timer_fire;
assign timer_fire = timer_enabled && (timer_cnt == 32'b0);

reg [31:0] csr_estat_next;
always @(*) begin
    csr_estat_next = csr_estat;

    // Hardware interrupt lines are sampled every cycle.
    csr_estat_next[9:2] = hw_int;

    // Timer interrupt pending bit is set by the timer and may be cleared by TICLR.
    if (timer_fire) begin
        csr_estat_next[11] = 1'b1;
    end

    if (csr_do_update) begin
        if (csr_exc_taken) begin
            csr_estat_next[21:16] = csr_exc_ecode;
            csr_estat_next[30:22] = csr_exc_esubcode;
        end
        else if (csr_we_commit) begin
            case (csr_num_commit)
                CSR_ESTAT: begin
                    csr_estat_next[1:0] = csr_wdata_commit[1:0];
                end
                CSR_TICLR: begin
                    if (csr_wdata_commit[0]) begin
                        csr_estat_next[11] = 1'b0;
                    end
                end
                default: ;
            endcase
        end
    end
end

always @(posedge clk) begin
    if (reset) begin
        csr_estat <= 32'b0;
    end
    else begin
        csr_estat <= csr_estat_next;
    end
end

always @(posedge clk) begin
    if (reset) begin
        // DA=1, PG=0, PLV=0, IE=0.  This is the common initial direct-address state.
        csr_crmd       <= 32'h0000_0008;
        csr_prmd       <= 32'b0;
        csr_ecfg       <= 32'b0;
        csr_era        <= 32'b0;
        csr_badv       <= 32'b0;
        csr_eentry     <= 32'b0;
        csr_save0      <= 32'b0;
        csr_save1      <= 32'b0;
        csr_save2      <= 32'b0;
        csr_save3      <= 32'b0;
        csr_tid        <= 32'b0;
        csr_tcfg       <= 32'b0;
        csr_llbctl     <= 32'b0;
        csr_tlbidx     <= 32'b0;
        csr_tlbehi     <= 32'b0;
        csr_tlbelo0    <= 32'b0;
        csr_tlbelo1    <= 32'b0;
        csr_asid       <= {8'b0, 8'd10, 6'b0, 10'b0};
        csr_tlbrentry  <= 32'b0;
        csr_dmw0       <= 32'b0;
        csr_dmw1       <= 32'b0;
        tlbfill_index <= 5'b0;
        timer_cnt      <= 32'hffff_ffff;
        stable_counter <= 64'b0;
    end
    else begin
        stable_counter <= stable_counter + 64'b1;

        if (ll_w_commit) begin
            csr_llbctl[0] <= 1'b1;
        end
        else if (sc_w_commit) begin
            csr_llbctl[0] <= 1'b0;
        end
        // ESTAT is updated in a separate next-state block for timing.

        // Timer countdown and timer interrupt generation. ESTAT.IS[11] is cleared by TICLR.CLR.
        if (timer_enabled) begin
            if (timer_cnt == 32'b0) begin
                if (timer_periodic) begin
                    timer_cnt <= timer_init_value;
                end
                else begin
                    csr_tcfg[0] <= 1'b0;
                    timer_cnt   <= 32'hffff_ffff;
                end
            end
            else begin
                timer_cnt <= timer_cnt - 32'b1;
            end
        end

        if (csr_do_update) begin
            if (csr_exc_taken) begin
                csr_prmd[1:0]    <= csr_crmd[1:0];  // PPLV <= PLV
                csr_prmd[2]      <= csr_crmd[2];    // PIE  <= IE
                csr_crmd[1:0]    <= 2'b0;           // enter kernel mode
                csr_crmd[2]      <= 1'b0;           // disable interrupt
                if (csr_exc_tlbr) begin
                    // TLB refill handler runs in direct-address mode.
                    csr_crmd[3] <= 1'b1;            // DA
                    csr_crmd[4] <= 1'b0;            // PG
                end
                csr_era          <= csr_pc;
                if (csr_exc_adef | csr_exc_ale | csr_exc_tlb_related) begin
                    csr_badv <= csr_exc_badv;
                end
                if (csr_exc_tlb_related) begin
                    csr_tlbehi <= {csr_exc_badv[31:13], 13'b0};
                end
            end
            else if (csr_inst_ertn) begin
                if (!csr_llbctl[2]) begin
                    csr_llbctl[0] <= 1'b0;
                end
                csr_llbctl[2] <= 1'b0;
                csr_crmd[1:0] <= csr_prmd[1:0];
                csr_crmd[2]   <= csr_prmd[2];
                if (csr_estat[21:16] == 6'h3f) begin
                    // Return from TLB refill back to paging mode.
                    csr_crmd[3] <= 1'b0;            // DA
                    csr_crmd[4] <= 1'b1;            // PG
                end
            end
            else if (csr_inst_tlbsrch) begin
                if (csr_tlbsrch_found) begin
                    csr_tlbidx <= {1'b0, csr_tlbidx[30:5], csr_tlbsrch_index};
                end
                else begin
                    csr_tlbidx[31] <= 1'b1;
                end
            end
            else if (csr_inst_tlbrd) begin
                if (csr_tlbrd_e) begin
                    csr_tlbidx <= {1'b0, 1'b0, csr_tlbrd_ps, 19'b0, csr_tlbidx[4:0]};
                    csr_tlbehi  <= {csr_tlbrd_vppn, 13'b0};
                    csr_tlbelo0 <= {4'b0, csr_tlbrd_ppn0, 1'b0, csr_tlbrd_g, csr_tlbrd_mat0, csr_tlbrd_plv0, csr_tlbrd_d0, csr_tlbrd_v0};
                    csr_tlbelo1 <= {4'b0, csr_tlbrd_ppn1, 1'b0, csr_tlbrd_g, csr_tlbrd_mat1, csr_tlbrd_plv1, csr_tlbrd_d1, csr_tlbrd_v1};
                    csr_asid    <= {8'b0, 8'd10, 6'b0, csr_tlbrd_asid};
                end
                else begin
                    csr_tlbidx <= {1'b1, 1'b0, 6'b0, 19'b0, csr_tlbidx[4:0]};
                    csr_tlbehi  <= 32'b0;
                    csr_tlbelo0 <= 32'b0;
                    csr_tlbelo1 <= 32'b0;
                    csr_asid    <= {8'b0, 8'd10, 6'b0, 10'b0};
                end
            end
            else if (csr_inst_tlbfill) begin
                tlbfill_index <= tlbfill_index + 5'b1;
            end
            else if (csr_we_commit) begin
                case (csr_num_commit)
                    CSR_CRMD  : csr_crmd   <= csr_wdata_commit;
                    CSR_PRMD  : csr_prmd   <= csr_wdata_commit;
                    CSR_ECFG  : begin
                        // ECFG.LIE has valid interrupt-enable bits [12:0] except bit10,
                        // and upper bits are reserved.  Bit10 must read as 0.
                        // The exp13 timer-interrupt test writes 0x1fff and expects
                        // the stored/read-back value to be 0x1bff.
                        csr_ecfg <= {19'b0, csr_wdata_commit[12:11], 1'b0, csr_wdata_commit[9:0]};
                    end
                    CSR_ESTAT : ;
                    CSR_ERA   : csr_era    <= csr_wdata_commit;
                    CSR_BADV  : csr_badv   <= csr_wdata_commit;
                    CSR_EENTRY: csr_eentry <= csr_wdata_commit;
                    CSR_SAVE0 : csr_save0  <= csr_wdata_commit;
                    CSR_SAVE1 : csr_save1  <= csr_wdata_commit;
                    CSR_SAVE2 : csr_save2  <= csr_wdata_commit;
                    CSR_SAVE3 : csr_save3  <= csr_wdata_commit;
                    CSR_TLBIDX: csr_tlbidx <= {csr_wdata_commit[31], 1'b0, csr_wdata_commit[29:24], 19'b0, csr_wdata_commit[4:0]};
                    CSR_TLBEHI: csr_tlbehi <= {csr_wdata_commit[31:13], 13'b0};
                    CSR_TLBELO0: csr_tlbelo0 <= {4'b0, csr_wdata_commit[27:8], 1'b0, csr_wdata_commit[6:0]};
                    CSR_TLBELO1: csr_tlbelo1 <= {4'b0, csr_wdata_commit[27:8], 1'b0, csr_wdata_commit[6:0]};
                    CSR_ASID  : csr_asid   <= {8'b0, 8'd10, 6'b0, csr_wdata_commit[9:0]};
                    CSR_TLBRENTRY: csr_tlbrentry <= {csr_wdata_commit[31:6], 6'b0};
                    CSR_DMW0  : csr_dmw0 <= csr_wdata_commit;
                    CSR_DMW1  : csr_dmw1 <= csr_wdata_commit;
                    CSR_TID   : csr_tid    <= csr_wdata_commit;
                    CSR_LLBCTL: begin
                        if (csr_wdata_commit[1]) begin
                            csr_llbctl[0] <= 1'b0;
                        end
                        csr_llbctl[2] <= csr_wdata_commit[2];
                    end
                    CSR_TCFG  : begin
                        csr_tcfg  <= csr_wdata_commit;
                        timer_cnt <= csr_wdata_commit[0] ? {csr_wdata_commit[31:2], 2'b0} : 32'hffff_ffff;
                    end
                    CSR_TICLR : ;
                    default   : ;
                endcase
            end
        end
    end
end

// Phase 4 timing cut: split the normal refill and the dual-issue refill into
// two independent payload banks.  In Phase 3, dual_pair_fire selected between
// fs_to_ds_bus and fs_to_ds_bus1 directly on every bit of this 98-bit register.
// The routed report therefore contained a 16-LUT self path:
//   current ID payload -> exception/pairing logic -> wide refill mux -> ID payload
// Only the 1-bit bank selector needs to depend on dual_pair_fire.  The two wide
// banks see direct IF payload data and a common ds_allowin clock enable.
(* keep = "true", max_fanout = 8 *) wire [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_single_r;
(* keep = "true", max_fanout = 8 *) wire [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_dual_r;
(* keep = "true", max_fanout = 8 *) reg                         fs_to_ds_bus_sel_r;
wire [`FS_TO_DS_BUS_WD -1:0] fs_to_ds_bus_r;

assign fs_to_ds_bus_r = fs_to_ds_bus_sel_r ? fs_to_ds_bus_dual_r
                                             : fs_to_ds_bus_single_r;

assign {ds_slot0_dual_static_ok,
        ds_pred_taken,
        ds_pred_nextpc,
        ds_inst,
        ds_pc  } = fs_to_ds_bus_r;

assign {rf_we   ,  //37:37
        rf_waddr,  //36:32
        rf_wdata   //31:0
       } = ws_to_rf_bus;

wire [`DS_TO_ES_BUS_WD-1:0] ds_to_es_bus_normal;
wire [`DS_TO_ES_BUS_WD-1:0] ds_to_es_bus_swapped_mem;
wire [`DS_TO_ES_BUS_WD-1:0] ds_to_es_bus_swapped_mul;
assign ds_to_es_bus_normal = {1'b0,             // main lane carries older instruction
                         ds_inst,           // 32, instruction for Difftest
                         deferred_load_branch_candidate, // 1, resolve condition in EXE
                         deferred_load_branch_candidate &&
                             es_to_ds_mem_load_op && br_rj_es_match, // 1
                         deferred_load_branch_candidate &&
                             es_to_ds_mem_load_op && br_rd_es_match, // 1
                         deferred_load_branch_candidate && ds_pred_taken, // 1
                         deferred_load_branch_candidate ? ds_pred_nextpc : 32'b0, // 32
                         data_uncached_id, // 1, data access should bypass DCache
                       direct_addr_exe_bypass, // 1, use EXE ALU rj+imm as physical address
                       dmw_addr_exe_bypass, // 1, replace EXE effective-address segment
                       inst_cacop   ,   // 1, exp23 CACOP operation
                       rd           ,   // 5, CACOP code field
                       dmw_addr_exe_bypass ? {dmw_pseg_to_es, 29'b0} :
                                                   mem_addr_phy_to_es,
                       muldiv_op    ,   // 7, exp10 mul/div/mod op
                       res_from_csr ,   // 1, exp12 CSR result select
                       csr_rvalue   ,   // 32, old CSR value for csrrd/csrwr/csrxchg
                       alu_op       ,   // 12
                       mem_op       ,   // 8, exp11 load/store type
                       src1_is_pc   ,   // 1
                       src2_is_imm  ,   // 1
                       src2_is_4    ,   // 1
                       gr_we        ,   // 1
                       mem_we       ,   // 1
                       dest         ,   // 5
                       imm_to_es    ,   // 32
                       // Predecoded late-forward selectors.  This replaces the
                       // old {rj,rk,rd,valid} payload and shrinks the bus by 12 bits.
                       rj_from_next_ms,  // 1: producer is in MEM next cycle
                       rk_from_next_ms,  // 1
                       rd_from_next_ms,  // 1
                       rj_from_next_ws,  // 1: released load is in WB next cycle
                       rk_from_next_ws,  // 1
                       rd_from_next_ws,  // 1
                       rj_value_to_es,   // 32
                       rkd_value    ,   // 32
                       ds_pc        ,    // 32
                       res_from_mem
                    };

`ifdef DISABLE_M16_LANE_SWAP
assign ds_to_es_bus_swapped_mem = {`DS_TO_ES_BUS_WD{1'b0}};
`else
assign ds_to_es_bus_swapped_mem = {1'b1,                // main lane is younger
                         slot1_candidate_inst,
                         1'b0, 1'b0, 1'b0, 1'b0, 32'b0, // no deferred branch
                         swap_candidate_uncached,
                         swap_candidate_direct_bypass,   // direct: EXE rj+imm; DMW: captured PA
                         1'b0,                           // DMW PA was captured in ID
                         1'b0,                           // no CACOP
                         5'b0,
                         swap_candidate_phy_addr,
                         7'b0,                           // no mul/div
                         1'b0,                           // no CSR result
                         32'b0,
                         12'b0000_0000_0001,             // address add
                         slot1_predecoded_mem_op,
                         1'b0,                           // src1 is register
                         1'b1,                           // src2 is immediate
                         1'b0,
                         slot1_candidate_load,
                         slot1_candidate_store,
                         slot1_candidate_load ? slot1_rd : 5'b0,
                         {{20{slot1_i12[11]}}, slot1_i12},
                         swap_rj_from_next_ms,
                         1'b0,
                         swap_rd_from_next_ms,
                         1'b0,
                         1'b0,
                         1'b0,
                         swap_candidate_base,
                         swap_candidate_store_data,
                         slot1_candidate_pc,
                         slot1_candidate_load
                    };

`endif

`ifdef DISABLE_M162_ALU_MUL_SWAP
assign ds_to_es_bus_swapped_mul = {`DS_TO_ES_BUS_WD{1'b0}};
`else
assign ds_to_es_bus_swapped_mul = {1'b1,                // main lane is younger
                         slot1_candidate_inst,
                         1'b0, 1'b0, 1'b0, 1'b0, 32'b0, // no deferred branch
                         1'b0,                           // cached/non-memory
                         1'b0,                           // no direct address bypass
                         1'b0,                           // no DMW address bypass
                         1'b0,                           // no CACOP
                         5'b0,
                         32'b0,                          // no physical address payload
                         slot1_muldiv_op,                // existing main multiplier
                         1'b0,                           // no CSR result
                         32'b0,
                         12'b0,                          // ALU result unused
                         8'b0,                           // no memory operation
                         1'b0,
                         1'b0,
                         1'b0,
                         1'b1,                           // multiply writes rd
                         1'b0,
                         slot1_rd,
                         32'b0,
                         mul_swap_rj_from_next_ms,
                         mul_swap_rk_from_next_ms,
                         1'b0,
                         mul_swap_rj_from_next_ws,
                         mul_swap_rk_from_next_ws,
                         1'b0,
                         slot1_rj_value,
                         slot1_rk_value,
                         slot1_candidate_pc,
                         1'b0
                    };

`endif

assign ds_to_es_bus = pair_swap_mem_fire ? ds_to_es_bus_swapped_mem :
                      pair_swap_mul_fire ? ds_to_es_bus_swapped_mul :
                                           ds_to_es_bus_normal;

// With forwarding, ID only stalls on load-use hazards.  System/CSR/TLB/exception
// instructions are swallowed into sys_* registers and committed later, so the
// current sys_stage_inst decode no longer participates in ds_allowin.  This cuts
// the self path fs_to_ds_bus_r -> TLB/exception decode -> fs_to_ds_bus_r CE.
assign ds_ready_go    = sys_stage_valid ? 1'b0 : normal_ready_go;
assign ds_allowin     = !ds_valid || ds_ready_go && es_allowin;

// M19.7 physical structure: the same architectural capture condition is
// materialized as eight local cones instead of one device-spanning CE net for
// both 98-bit payload banks.  The routed M19.3 limiter spent 81% of its delay
// in interconnect and ended on this CE.  Keeping the equivalent cones distinct
// lets each quarter-bank remain local without changing the handshake or adding
// a pipeline cycle.
(* keep = "true" *) wire ds_cap_single_q0 = !ds_valid ||
                                             (ds_ready_go && es_allowin);
(* keep = "true" *) wire ds_cap_single_q1 = !ds_valid ||
                                             (ds_ready_go && es_allowin);
(* keep = "true" *) wire ds_cap_single_q2 = !ds_valid ||
                                             (ds_ready_go && es_allowin);
(* keep = "true" *) wire ds_cap_single_q3 = !ds_valid ||
                                             (ds_ready_go && es_allowin);
(* keep = "true" *) wire ds_cap_dual_q0   = !ds_valid ||
                                             (ds_ready_go && es_allowin);
(* keep = "true" *) wire ds_cap_dual_q1   = !ds_valid ||
                                             (ds_ready_go && es_allowin);
(* keep = "true" *) wire ds_cap_dual_q2   = !ds_valid ||
                                             (ds_ready_go && es_allowin);
(* keep = "true" *) wire ds_cap_dual_q3   = !ds_valid ||
                                             (ds_ready_go && es_allowin);

// M19.9 physical timing cut:
// Vivado absorbed the inferred payload-register enables into per-bit D-input
// feedback muxes.  The routed M19.7 worst path therefore paid another LUT and
// a long final data route after the already-expensive ID allow-in cone.  Use
// explicit 7-series FDREs so the identical synchronous enable is implemented
// on the CE pin and each payload bit keeps a direct IF data input.  Reset,
// enable and sampled values remain bit-for-bit identical.
genvar ds_payload_q0_i;
genvar ds_payload_q1_i;
genvar ds_payload_q2_i;
genvar ds_payload_q3_i;
generate
    for (ds_payload_q0_i = 0; ds_payload_q0_i < 25;
         ds_payload_q0_i = ds_payload_q0_i + 1) begin : gen_ds_payload_q0
        FDRE #(.INIT(1'b0)) u_single_ff (
            .Q  (fs_to_ds_bus_single_r[ds_payload_q0_i]),
            .C  (clk),
            .CE (ds_cap_single_q0),
            .D  (fs_to_ds_bus[ds_payload_q0_i]),
            .R  (reset)
        );
        FDRE #(.INIT(1'b0)) u_dual_ff (
            .Q  (fs_to_ds_bus_dual_r[ds_payload_q0_i]),
            .C  (clk),
            .CE (ds_cap_dual_q0),
            .D  (fs_to_ds_bus1[ds_payload_q0_i]),
            .R  (reset)
        );
    end
    for (ds_payload_q1_i = 25; ds_payload_q1_i < 50;
         ds_payload_q1_i = ds_payload_q1_i + 1) begin : gen_ds_payload_q1
        FDRE #(.INIT(1'b0)) u_single_ff (
            .Q  (fs_to_ds_bus_single_r[ds_payload_q1_i]),
            .C  (clk),
            .CE (ds_cap_single_q1),
            .D  (fs_to_ds_bus[ds_payload_q1_i]),
            .R  (reset)
        );
        FDRE #(.INIT(1'b0)) u_dual_ff (
            .Q  (fs_to_ds_bus_dual_r[ds_payload_q1_i]),
            .C  (clk),
            .CE (ds_cap_dual_q1),
            .D  (fs_to_ds_bus1[ds_payload_q1_i]),
            .R  (reset)
        );
    end
    for (ds_payload_q2_i = 50; ds_payload_q2_i < 74;
         ds_payload_q2_i = ds_payload_q2_i + 1) begin : gen_ds_payload_q2
        FDRE #(.INIT(1'b0)) u_single_ff (
            .Q  (fs_to_ds_bus_single_r[ds_payload_q2_i]),
            .C  (clk),
            .CE (ds_cap_single_q2),
            .D  (fs_to_ds_bus[ds_payload_q2_i]),
            .R  (reset)
        );
        FDRE #(.INIT(1'b0)) u_dual_ff (
            .Q  (fs_to_ds_bus_dual_r[ds_payload_q2_i]),
            .C  (clk),
            .CE (ds_cap_dual_q2),
            .D  (fs_to_ds_bus1[ds_payload_q2_i]),
            .R  (reset)
        );
    end
    for (ds_payload_q3_i = 74; ds_payload_q3_i < `FS_TO_DS_BUS_WD;
         ds_payload_q3_i = ds_payload_q3_i + 1) begin : gen_ds_payload_q3
        FDRE #(.INIT(1'b0)) u_single_ff (
            .Q  (fs_to_ds_bus_single_r[ds_payload_q3_i]),
            .C  (clk),
            .CE (ds_cap_single_q3),
            .D  (fs_to_ds_bus[ds_payload_q3_i]),
            .R  (reset)
        );
        FDRE #(.INIT(1'b0)) u_dual_ff (
            .Q  (fs_to_ds_bus_dual_r[ds_payload_q3_i]),
            .C  (clk),
            .CE (ds_cap_dual_q3),
            .D  (fs_to_ds_bus1[ds_payload_q3_i]),
            .R  (reset)
        );
    end
endgenerate

// exp19 ref-local v9 trace fix:
// CSRWR/CSRXCHG have two effects:
//   1) write old CSR value back to GPR rd, which must still pass through EXE/MEM/WB
//      so the golden trace sees PC/wnum/wdata for the CSR instruction;
//   2) update the CSR side effect, which remains staged in sys_*_r and is committed
//      only after older pipeline stages drain.
// v6-v8 incorrectly swallowed CSRWR/CSRXCHG from ds_to_es_valid, so the CSR old-value
// writeback disappeared and the next visible writeback came after ERTN.
assign ds_to_es_valid = ds_valid && ds_ready_go &&
                        ((!sys_stage_inst) || csr_we) &&
                        !exc_taken && !inst_ertn && !external_flush;
always @(posedge clk) begin
    if (reset) begin
        ds_valid <= 1'b0;
        fs_to_ds_bus_sel_r    <= 1'b0;
        sys_stage_valid <= 1'b0;
        sys_invtlb_active_r <= 1'b0;
        sys_csr_we_r <= 1'b0;
        sys_csr_num_r <= 14'b0;
        sys_csr_wdata_r <= 32'b0;
        ds_tlb_checked_r <= 1'b0;
        ds_tlb_lookup_r <= 1'b0;
        ds_tlb_exc_badv_r <= 32'b0;
        ds_tlb_exc_tlbr_r <= 1'b0;
        ds_tlb_exc_pif_r <= 1'b0;
        ds_tlb_exc_pil_r <= 1'b0;
        ds_tlb_exc_pis_r <= 1'b0;
        ds_tlb_exc_pme_r <= 1'b0;
        ds_tlb_exc_ppi_r <= 1'b0;
        ds_pre_inst_need_tlb_r <= 1'b0;
        ds_pre_data_need_tlb_r <= 1'b0;
        ds_pre_exc_adef_r <= 1'b0;
        ds_pre_exc_ale_r <= 1'b0;
        ds_pre_mem_store_r <= 1'b0;
        ds_pre_tlb_read_like_r <= 1'b0;
        ds_pre_tlb_access_like_r <= 1'b0;
        ds_pre_direct_addr_mode_r <= 1'b0;
        ds_pre_data_dmw_hit_r <= 1'b0;
        ds_pre_crmd_plv_r <= 2'b0;
        ds_pre_pc_r <= 32'b0;
        ds_pre_mem_addr_id_r <= 32'b0;
        ds_pre_data_dmw_paddr_r <= 32'b0;
        ds_tlb_s0_vppn_r <= 19'b0;
        ds_tlb_s0_va_bit12_r <= 1'b0;
        ds_tlb_s0_asid_r <= 10'b0;
        ds_tlb_s1_vppn_r <= 19'b0;
        ds_tlb_s1_va_bit12_r <= 1'b0;
        ds_tlb_s1_asid_r <= 10'b0;
        ds_pre_tlb_s0_found_r <= 1'b0;
        ds_pre_tlb_s0_v_r <= 1'b0;
        ds_pre_tlb_s0_plv_fail_r <= 1'b0;
        ds_pre_tlb_s1_found_r <= 1'b0;
        ds_pre_tlb_s1_index_r <= 4'b0;
        ds_pre_tlb_s1_ppn_r <= 20'b0;
        ds_pre_tlb_s1_ps_r <= 6'b0;
        ds_pre_tlb_s1_mat_r <= MAT_CC;
        ds_pre_tlb_s1_v_r <= 1'b0;
        ds_pre_tlb_s1_d_r <= 1'b0;
        ds_pre_tlb_s1_plv_fail_r <= 1'b0;
        ds_mem_addr_phy_r <= 32'b0;
    end
    else begin
        // Frequency cut for the ID/MMU precheck path.
        //
        // The old implementation updated the wide precheck/result registers only
        // under ds_tlb_lookup_fire/ds_tlb_classify_fire.  Vivado therefore built
        // a very large clock-enable cone containing instruction decode, RAW hazard,
        // branch recovery, CSR/system decode and MMU controls.  At 92.857 MHz this
        // cone drove hundreds of CE endpoints and dominated both WNS and TNS.
        //
        // These registers are architecturally meaningful only while the two-bit
        // lookup protocol below says LOOKUP/DONE.  It is therefore safe to let the
        // payload registers track their combinational sources every cycle and use
        // ds_tlb_lookup_r/ds_tlb_checked_r as the sole validity state.  This turns
        // the former wide CE cone into ordinary, much shorter D paths.
        ds_pre_inst_need_tlb_r    <= inst_need_tlb;
        ds_pre_data_need_tlb_r    <= data_need_tlb;
        ds_pre_exc_adef_r         <= exc_adef;
        ds_pre_exc_ale_r          <= exc_ale;
        ds_pre_mem_store_r        <= ds_mem_store;
        ds_pre_tlb_read_like_r    <= ds_tlb_read_like;
        ds_pre_tlb_access_like_r  <= ds_tlb_access_like;
        ds_pre_direct_addr_mode_r <= direct_addr_mode;
        ds_pre_data_dmw_hit_r     <= data_dmw_hit;
        ds_pre_crmd_plv_r         <= crmd_plv;
        ds_pre_pc_r               <= ds_pc;
        ds_pre_mem_addr_id_r      <= mem_addr_id;
        ds_pre_data_dmw_paddr_r   <= data_dmw_paddr;
        ds_tlb_s0_vppn_r          <= ds_pc[31:13];
        ds_tlb_s0_va_bit12_r      <= ds_pc[12];
        ds_tlb_s0_asid_r          <= csr_asid[9:0];
        ds_tlb_s1_vppn_r          <= tlb_s1_vppn;
        ds_tlb_s1_va_bit12_r      <= tlb_s1_va_bit12;
        ds_tlb_s1_asid_r          <= tlb_s1_asid;

        // The TLB search result is likewise sampled continuously.  It is consumed
        // only after ds_tlb_classify_fire sets ds_tlb_checked_r, so values observed
        // outside a valid lookup are don't-care and cannot affect architectural
        // state.
        ds_pre_tlb_s0_found_r     <= tlb_s0_found;
        ds_pre_tlb_s0_v_r         <= tlb_s0_v;
        ds_pre_tlb_s0_plv_fail_r  <= ds_lookup_tlb_s0_plv_fail;
        ds_pre_tlb_s1_found_r     <= tlb_s1_found;
        ds_pre_tlb_s1_index_r     <= tlb_s1_index;
        ds_pre_tlb_s1_ppn_r       <= tlb_s1_ppn;
        ds_pre_tlb_s1_ps_r        <= tlb_s1_ps;
        ds_pre_tlb_s1_mat_r       <= tlb_s1_mat;
        ds_pre_tlb_s1_v_r         <= tlb_s1_v;
        ds_pre_tlb_s1_d_r         <= tlb_s1_d;
        ds_pre_tlb_s1_plv_fail_r  <= ds_lookup_tlb_s1_plv_fail;
        ds_tlb_exc_badv_r         <= ds_lookup_exc_tlb_badv;
        ds_tlb_exc_tlbr_r         <= ds_lookup_exc_tlbr;
        ds_tlb_exc_pif_r          <= ds_lookup_exc_pif;
        ds_tlb_exc_pil_r          <= ds_lookup_exc_pil;
        ds_tlb_exc_pis_r          <= ds_lookup_exc_pis;
        ds_tlb_exc_pme_r          <= ds_lookup_exc_pme;
        ds_tlb_exc_ppi_r          <= ds_lookup_exc_ppi;
        // ds_mem_addr_phy_r is now dedicated to registered TLB lookup output.
        // DMW-only accesses consume ds_pre_data_dmw_paddr_r directly and direct
        // addressing is generated in EXE, so no current-ID effective address
        // needs to reach this register bank.
        ds_mem_addr_phy_r <= ds_lookup_mem_addr_phy;

        // Only the two validity/state bits retain conditional next-state logic.
        // Priority matches the original implementation.
        // Normal branch recovery is already registered in mycpu_core.  Use that
        // registered external_flush here instead of feeding the long current-ID
        // compare/target cone back into these state flops.  ds_to_es_valid is
        // gated by external_flush, so the temporarily latched wrong-path entry
        // cannot enter EXE before being killed.
        if (external_flush) begin
            ds_tlb_checked_r <= 1'b0;
            ds_tlb_lookup_r  <= 1'b0;
        end
        else if (ds_fast_addr_capture_fire) begin
            // One registered address cycle, then the direct/DMW access may enter EXE.
            ds_tlb_checked_r <= 1'b1;
            ds_tlb_lookup_r  <= 1'b0;
        end
        else if (ds_tlb_lookup_fire) begin
            ds_tlb_checked_r <= 1'b0;
            ds_tlb_lookup_r  <= 1'b1;
        end
        else if (ds_tlb_classify_fire) begin
            ds_tlb_checked_r <= 1'b1;
            ds_tlb_lookup_r  <= 1'b0;
        end
        else if (ds_allowin) begin
            ds_tlb_checked_r <= 1'b0;
            ds_tlb_lookup_r  <= 1'b0;
        end

        if (external_flush) begin
            sys_stage_valid <= 1'b0;
            sys_invtlb_active_r <= 1'b0;
        end
        else if (sys_commit_fire) begin
            sys_stage_valid <= 1'b0;
            sys_invtlb_active_r <= 1'b0;
        end
        else if (sys_stage_start) begin
            sys_stage_valid <= 1'b1;
            // On this edge sys_inst_invtlb_r captures the same predicate, so
            // this is cycle-equivalent to sys_stage_valid && sys_inst_invtlb_r.
            sys_invtlb_active_r <= inst_invtlb_valid_op;
        end

        // M19 timing cut: while no serialized system operation is resident,
        // continuously sample the candidate payload.  sys_stage_valid is the
        // only architectural validity bit, so the payload freezes on the same
        // edge that raises it and remains stable until commit.  This replaces
        // the deep sys_capture_fire clock-enable cone with !sys_stage_valid.
        if (!sys_stage_valid) begin
            sys_exc_taken_r       <= exc_taken;
            sys_exc_ecode_r       <= exc_ecode;
            sys_exc_esubcode_r    <= exc_esubcode;
            sys_exc_badv_r        <= exc_badv;
            sys_exc_tlbr_r        <= exc_tlbr_sel;
            sys_exc_adef_r        <= exc_adef;
            sys_exc_ale_r         <= exc_ale;
            sys_exc_tlb_related_r <= exc_tlb_related_sel;
            sys_pc_r              <= ds_pc;
            sys_inst_r            <= ds_inst;
            sys_inst_ertn_r       <= inst_ertn;
            sys_inst_tlbsrch_r    <= inst_tlbsrch;
            sys_inst_tlbrd_r      <= inst_tlbrd;
            sys_inst_tlbwr_r      <= inst_tlbwr;
            sys_inst_tlbfill_r    <= inst_tlbfill;
            sys_inst_invtlb_r     <= inst_invtlb_valid_op;
            sys_invtlb_op_r       <= invtlb_op;
            sys_invtlb_vppn_r     <= rkd_value[31:13];
            sys_invtlb_va_bit12_r <= rkd_value[12];
            sys_invtlb_asid_r     <= rj_value[9:0];
            sys_tlb_s1_found_r    <= (ds_tlb_check_needed && ds_tlb_checked_r) ? ds_pre_tlb_s1_found_r : tlb_s1_found;
            sys_tlb_s1_index_r    <= (ds_tlb_check_needed && ds_tlb_checked_r) ? ds_pre_tlb_s1_index_r : tlb_s1_index;
            sys_tlb_r_e_r         <= tlb_r_e;
            sys_tlb_r_vppn_r      <= tlb_r_vppn;
            sys_tlb_r_ps_r        <= tlb_r_ps;
            sys_tlb_r_asid_r      <= tlb_r_asid;
            sys_tlb_r_g_r         <= tlb_r_g;
            sys_tlb_r_ppn0_r      <= tlb_r_ppn0;
            sys_tlb_r_plv0_r      <= tlb_r_plv0;
            sys_tlb_r_mat0_r      <= tlb_r_mat0;
            sys_tlb_r_d0_r        <= tlb_r_d0;
            sys_tlb_r_v0_r        <= tlb_r_v0;
            sys_tlb_r_ppn1_r      <= tlb_r_ppn1;
            sys_tlb_r_plv1_r      <= tlb_r_plv1;
            sys_tlb_r_mat1_r      <= tlb_r_mat1;
            sys_tlb_r_d1_r        <= tlb_r_d1;
            sys_tlb_r_v1_r        <= tlb_r_v1;
            sys_tlbfill_index_r   <= tlbfill_index;
            sys_csr_we_r          <= csr_we;
            sys_csr_num_r         <= csr_num;
            sys_csr_wdata_r       <= csr_wdata;
        end

        if (external_flush) begin
            ds_valid <= 1'b0;
        end
        else if (ds_allowin) begin
            // On a dual issue, IF head is consumed by Slot 1. Refill Slot 0
            // from the second FIFO entry; otherwise keep single-issue behavior.
            ds_valid <= pair_refill_select ? fs_to_ds_valid1 : fs_to_ds_valid;
        end

        if (ds_allowin) begin
            // The selector chooses the architecturally correct refill after the
            // edge.  Invalid candidate payloads remain intentional don't-cares.
            fs_to_ds_bus_sel_r    <= pair_refill_select && fs_to_ds_valid1;
        end
    end
end

`ifdef DIFFTEST_EN
    // Synchronous exceptions do not arrive at WB as InstrCommit packets.
    // Delay the DPI packet one cycle so exception CSR updates are visible.
    reg        dt_excp_valid;
    reg        dt_excp_eret;
    reg [31:0] dt_excp_intr_no;
    reg [31:0] dt_excp_cause;
    reg [31:0] dt_excp_pc;
    reg [31:0] dt_excp_inst;

    always @(posedge clk) begin
        if (reset) begin
            dt_excp_valid   <= 1'b0;
            dt_excp_eret    <= 1'b0;
            dt_excp_intr_no <= 32'b0;
            dt_excp_cause   <= 32'b0;
            dt_excp_pc      <= 32'b0;
            dt_excp_inst    <= 32'b0;
        end
        else begin
            dt_excp_valid   <= sys_commit_fire && sys_exc_taken_r;
            dt_excp_eret    <= sys_commit_fire && sys_inst_ertn_r;
            dt_excp_intr_no <= {21'b0, csr_estat[12:2]};
            dt_excp_cause   <= {26'b0, sys_exc_ecode_r};
            dt_excp_pc      <= sys_pc_r;
            dt_excp_inst    <= sys_inst_r;
        end
    end

    DifftestExcpEvent u_difftest_excp_event (
        .clock         (clk),
        .coreid        (8'd0),
        .excp_valid    (dt_excp_valid),
        .eret          (1'b0),
        .intrNo        (dt_excp_intr_no),
        .cause         (dt_excp_cause),
        .exceptionPC   ({32'b0, dt_excp_pc}),
        .exceptionInst (dt_excp_inst)
    );

    // GPR snapshot. r0 is architecturally fixed to zero even though the
    // underlying array is not reset and may contain an arbitrary value.
    DifftestGRegState u_difftest_greg_state (
        .clock (clk),
        .coreid(8'd0),
        .gpr_0 (64'b0),
        .gpr_1 ({32'b0, u_regfile.rf[ 1]}),
        .gpr_2 ({32'b0, u_regfile.rf[ 2]}),
        .gpr_3 ({32'b0, u_regfile.rf[ 3]}),
        .gpr_4 ({32'b0, u_regfile.rf[ 4]}),
        .gpr_5 ({32'b0, u_regfile.rf[ 5]}),
        .gpr_6 ({32'b0, u_regfile.rf[ 6]}),
        .gpr_7 ({32'b0, u_regfile.rf[ 7]}),
        .gpr_8 ({32'b0, u_regfile.rf[ 8]}),
        .gpr_9 ({32'b0, u_regfile.rf[ 9]}),
        .gpr_10({32'b0, u_regfile.rf[10]}),
        .gpr_11({32'b0, u_regfile.rf[11]}),
        .gpr_12({32'b0, u_regfile.rf[12]}),
        .gpr_13({32'b0, u_regfile.rf[13]}),
        .gpr_14({32'b0, u_regfile.rf[14]}),
        .gpr_15({32'b0, u_regfile.rf[15]}),
        .gpr_16({32'b0, u_regfile.rf[16]}),
        .gpr_17({32'b0, u_regfile.rf[17]}),
        .gpr_18({32'b0, u_regfile.rf[18]}),
        .gpr_19({32'b0, u_regfile.rf[19]}),
        .gpr_20({32'b0, u_regfile.rf[20]}),
        .gpr_21({32'b0, u_regfile.rf[21]}),
        .gpr_22({32'b0, u_regfile.rf[22]}),
        .gpr_23({32'b0, u_regfile.rf[23]}),
        .gpr_24({32'b0, u_regfile.rf[24]}),
        .gpr_25({32'b0, u_regfile.rf[25]}),
        .gpr_26({32'b0, u_regfile.rf[26]}),
        .gpr_27({32'b0, u_regfile.rf[27]}),
        .gpr_28({32'b0, u_regfile.rf[28]}),
        .gpr_29({32'b0, u_regfile.rf[29]}),
        .gpr_30({32'b0, u_regfile.rf[30]}),
        .gpr_31({32'b0, u_regfile.rf[31]})
    );

    // Difftest samples on the same edge as sys_commit_fire.
    // For a CSRWR/CSRXCHG to EENTRY, export the architectural post-write
    // value instead of the register's pre-NBA value.
    wire [31:0] dt_csr_eentry =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_EENTRY))
        ? csr_wdata_commit
        : csr_eentry;

    // CRMD is updated with nonblocking assignment on sys_commit_fire.
    // Difftest samples at the same edge, so export the post-write value
    // for a CSRWR/CSRXCHG targeting CRMD.
    wire [31:0] dt_csr_crmd =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_CRMD))
        ? csr_wdata_commit
        : csr_crmd;

    // PRMD is updated with nonblocking assignment on sys_commit_fire.
    // Export the architectural post-write value for CSRWR/CSRXCHG to PRMD.
    wire [31:0] dt_csr_prmd =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_PRMD))
        ? csr_wdata_commit
        : csr_prmd;

    // ECFG masks reserved bits on write.  Export exactly the same
    // architectural post-write value at the Difftest sampling edge.
    wire [31:0] dt_csr_ecfg =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_ECFG))
        ? {19'b0, csr_wdata_commit[12:11], 1'b0,
           csr_wdata_commit[9:0]}
        : csr_ecfg;

    // ERA changes either through a normal CSR write or when an
    // exception commits. Export the post-write value at the Difftest edge.
    // TID is updated with nonblocking assignment on sys_commit_fire.
    // Export the architectural post-write value for CSRWR/CSRXCHG to TID.
    wire [31:0] dt_csr_tid =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_TID))
        ? csr_wdata_commit
        : csr_tid;

    // TCFG is updated with nonblocking assignment on sys_commit_fire.
    // Export the architectural post-write value for CSRWR/CSRXCHG to TCFG.
    wire [31:0] dt_csr_tcfg =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_TCFG))
        ? csr_wdata_commit
        : csr_tcfg;

    wire [31:0] dt_csr_era =
        // Normal CSRWR/CSRXCHG to ERA needs a same-edge post-write view.
        // Exception commit must NOT be forwarded here: the exception DPI
        // event is already delayed one cycle, while an older InstrCommit may
        // still be checked in the current cycle.
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_ERA))
        ? csr_wdata_commit
        : csr_era;

    // BADV is updated with a nonblocking assignment on sys_commit_fire.
    // For a normal CSRWR/CSRXCHG to BADV, Difftest must observe the
    // architectural post-write value at the same sampling edge.
    // Do not forward exception / ERTN updates here.
    wire [31:0] dt_csr_badv =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_BADV))
        ? csr_wdata_commit
        : csr_badv;

    // ASID is updated with a nonblocking assignment on sys_commit_fire.
    // For a normal CSRWR/CSRXCHG to ASID, Difftest must observe the
    // architectural post-write value at the same sampling edge.
    // Preserve the same ASID format used by the actual CSR write logic.
    // Do not forward exception / ERTN updates here.
    wire [31:0] dt_csr_asid =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_ASID))
        ? {8'b0, 8'd10, 6'b0, csr_wdata_commit[9:0]}
        : csr_asid;

    // TLBIDX is updated with a nonblocking assignment on sys_commit_fire.
    // For a normal CSRWR/CSRXCHG to TLBIDX, Difftest must observe the
    // architectural post-write value at the same sampling edge.
    // Preserve the same bit mask used by the actual CSR write logic.
    // Do not forward exception / ERTN / TLB-maintenance updates here.
    wire [31:0] dt_csr_tlbidx =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_TLBIDX))
        ? {csr_wdata_commit[31], 1'b0, csr_wdata_commit[29:24],
           19'b0, csr_wdata_commit[4:0]}
        : csr_tlbidx;

    // TLBEHI is updated with a nonblocking assignment on sys_commit_fire.
    // For a normal CSRWR/CSRXCHG to TLBEHI, Difftest must observe the
    // architectural post-write value at the same sampling edge.
    // Preserve the same bit mask used by the actual CSR write logic.
    // Do not forward exception / ERTN / TLB-maintenance updates here.
    wire [31:0] dt_csr_tlbehi =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_TLBEHI))
        ? {csr_wdata_commit[31:13], 13'b0}
        : csr_tlbehi;

    // TLBELO0 is updated with a nonblocking assignment on sys_commit_fire.
    // For a normal CSRWR/CSRXCHG to TLBELO0, Difftest must observe the
    // architectural post-write value at the same sampling edge.
    // Preserve the same bit mask used by the actual CSR write logic.
    // Do not forward exception / ERTN / TLB-maintenance updates here.
    wire [31:0] dt_csr_tlbelo0 =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_TLBELO0))
        ? {4'b0, csr_wdata_commit[27:8], 1'b0, csr_wdata_commit[6:0]}
        : csr_tlbelo0;

    // TLBELO1 is updated with a nonblocking assignment on sys_commit_fire.
    // For a normal CSRWR/CSRXCHG to TLBELO1, Difftest must observe the
    // architectural post-write value at the same sampling edge.
    // Preserve the same bit mask used by the actual CSR write logic.
    // Do not forward exception / ERTN / TLB-maintenance updates here.
    wire [31:0] dt_csr_tlbelo1 =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_TLBELO1))
        ? {4'b0, csr_wdata_commit[27:8], 1'b0, csr_wdata_commit[6:0]}
        : csr_tlbelo1;

    // TLBRENTRY is updated with a nonblocking assignment on sys_commit_fire.
    // Difftest samples at the same edge, so export the architectural
    // post-write value for a CSRWR/CSRXCHG targeting TLBRENTRY.
    wire [31:0] dt_csr_tlbrentry =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_TLBRENTRY))
        ? {csr_wdata_commit[31:6], 6'b0}
        : csr_tlbrentry;

    // DMW0 and DMW1 are updated with nonblocking assignments on sys_commit_fire.
    // Difftest samples at the same edge, so export the architectural post-write
    // value for a normal CSRWR/CSRXCHG targeting either DMW register.
    wire [31:0] dt_csr_dmw0 =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_DMW0))
        ? csr_wdata_commit
        : csr_dmw0;

// LLSC_STAGE62D_DONE
    wire [31:0] dt_csr_llbctl =
        ll_w_commit ? {29'b0, csr_llbctl[2], 1'b0, 1'b1} :
        sc_w_commit ? {29'b0, csr_llbctl[2], 2'b00} :
        (csr_do_update && csr_inst_ertn) ?
            {29'b0, 2'b00, (csr_llbctl[2] ? csr_llbctl[0] : 1'b0)} :
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn && csr_we_commit && (csr_num_commit == CSR_LLBCTL)) ?
            {29'b0, csr_wdata_commit[2], 1'b0, (csr_wdata_commit[1] ? 1'b0 : csr_llbctl[0])} :
        csr_llbctl;

    wire [31:0] dt_csr_dmw1 =
        (csr_do_update && !csr_exc_taken && !csr_inst_ertn &&
         csr_we_commit && (csr_num_commit == CSR_DMW1))
        ? csr_wdata_commit
        : csr_dmw1;

    DifftestCSRRegState u_difftest_csr_state (
        .clock    (clk),
        .coreid   (8'd0),
        .crmd     ({32'b0, dt_csr_crmd}),
        .prmd     ({32'b0, dt_csr_prmd}),
        .euen     (64'b0),
        .ecfg     ({32'b0, dt_csr_ecfg}),
        .estat    ({32'b0, csr_estat}),
        .era      ({32'b0, dt_csr_era}),
        .badv     ({32'b0, dt_csr_badv}),
        .eentry   ({32'b0, dt_csr_eentry}),
        .tlbidx   ({32'b0, dt_csr_tlbidx}),
        .tlbehi   ({32'b0, dt_csr_tlbehi}),
        .tlbelo0  ({32'b0, dt_csr_tlbelo0}),
        .tlbelo1  ({32'b0, dt_csr_tlbelo1}),
        .asid     ({32'b0, dt_csr_asid}),
        .pgdl     (64'b0),
        .pgdh     (64'b0),
        .save0    (64'b0),
        .save1    (64'b0),
        .save2    (64'b0),
        .save3    (64'b0),
        .tid      ({32'b0, dt_csr_tid}),
        .tcfg     ({32'b0, dt_csr_tcfg}),
        .tval     ({32'b0, csr_tval}),
        .ticlr    ({32'b0, csr_ticlr}),
        .llbctl   ({32'b0, dt_csr_llbctl}),
        .tlbrentry({32'b0, dt_csr_tlbrentry}),
        .dmw0     ({32'b0, dt_csr_dmw0}),
        .dmw1     ({32'b0, dt_csr_dmw1})
    );
`endif




endmodule




// =============================================================================
// Restricted dual-issue Slot-1 pipeline
// =============================================================================
module dual_issue_lane(
    input         clk,
    input         reset,
    input         external_flush,
    input         main_es_allowin,
    input         main_ms_allowin,
    input         main_ws_allowin,
    input         main_es_to_ms_valid,
    input         main_ms_to_ws_valid,
    input         issue_valid,
    input  [11:0] issue_alu_op,
    input  [31:0] issue_src1,
    input  [31:0] issue_src2,
    input  [ 4:0] issue_dest,
    input  [31:0] issue_pc,
    input  [31:0] issue_inst,
    input         issue_is_older,
    input         issue_src1_from_main_ms,
    input         issue_src2_from_main_ms,
    input         issue_src1_from_main_ws,
    input         issue_src2_from_main_ws,
    input         issue_src1_from_side_ms,
    input         issue_src2_from_side_ms,
    input  [31:0] main_ms_result,
    input  [31:0] main_ws_result,
    output [ 4:0] es_dest,
    output [ 4:0] ms_dest,
    output [31:0] ms_result,
    output        es_stage_valid,
    output        ms_stage_valid,
    output        ws_stage_valid,
    output        wb_we,
    output [ 4:0] wb_waddr,
    output [31:0] wb_wdata,
    output        wb_is_older,
    output [31:0] wb_pc
);

reg         es_valid;
reg  [11:0] es_alu_op;
reg  [31:0] es_src1;
reg  [31:0] es_src2;
reg         es_src1_from_main_ms;
reg         es_src2_from_main_ms;
reg         es_src1_from_main_ws;
reg         es_src2_from_main_ws;
reg         es_src1_from_side_ms;
reg         es_src2_from_side_ms;
reg  [ 4:0] es_dest_r;
reg  [31:0] es_pc;
reg  [31:0] es_inst;
reg         es_is_older;
wire [31:0] es_src1_final;
wire [31:0] es_src2_final;
wire [31:0] es_result;

reg         ms_valid;
reg  [31:0] ms_result_r;
reg  [ 4:0] ms_dest_r;
reg  [31:0] ms_pc;
reg  [31:0] ms_inst;
reg         ms_is_older;

reg         ws_valid;
reg  [31:0] ws_result_r;
reg  [ 4:0] ws_dest_r;
reg  [31:0] ws_pc;
reg  [31:0] ws_inst;
reg         ws_is_older;

// Late forwarding uses only registered MEM-stage data.  Main-MEM has priority
// by convention; a legal issue cannot match both current EXE destinations because
// WAW is prohibited for every issued pair.
assign es_src1_final = es_src1_from_main_ms ? main_ms_result :
                       es_src1_from_main_ws ? main_ws_result :
                       es_src1_from_side_ms ? ms_result_r : es_src1;
assign es_src2_final = es_src2_from_main_ms ? main_ms_result :
                       es_src2_from_main_ws ? main_ws_result :
                       es_src2_from_side_ms ? ms_result_r : es_src2;

alu u_slot1_alu(
    .alu_op     (es_alu_op),
    .alu_src1   (es_src1_final),
    .alu_src2   (es_src2_final),
    .alu_result (es_result)
);

assign es_dest = es_dest_r & {5{es_valid}};
assign ms_dest = ms_dest_r & {5{ms_valid}};
assign ms_result = ms_result_r;
assign es_stage_valid = es_valid;
assign ms_stage_valid = ms_valid;
assign ws_stage_valid = ws_valid;
assign wb_we    = ws_valid;
assign wb_waddr = ws_dest_r;
assign wb_wdata = ws_result_r;
assign wb_is_older = ws_valid && ws_is_older;
assign wb_pc = ws_pc;

always @(posedge clk) begin
    if (reset) begin
        es_valid    <= 1'b0;
        es_alu_op   <= 12'b0;
        es_src1     <= 32'b0;
        es_src2     <= 32'b0;
        es_src1_from_main_ms <= 1'b0;
        es_src2_from_main_ms <= 1'b0;
        es_src1_from_main_ws <= 1'b0;
        es_src2_from_main_ws <= 1'b0;
        es_src1_from_side_ms <= 1'b0;
        es_src2_from_side_ms <= 1'b0;
        es_dest_r   <= 5'b0;
        es_pc       <= 32'b0;
        es_inst     <= 32'b0;
        es_is_older<= 1'b0;
        ms_valid    <= 1'b0;
        ms_result_r <= 32'b0;
        ms_dest_r   <= 5'b0;
        ms_pc       <= 32'b0;
        ms_inst     <= 32'b0;
        ms_is_older<= 1'b0;
        ws_valid    <= 1'b0;
        ws_result_r <= 32'b0;
        ws_dest_r   <= 5'b0;
        ws_pc       <= 32'b0;
        ws_inst     <= 32'b0;
        ws_is_older<= 1'b0;
    end
    else begin
        // CACOP completion flushes only younger EXE work. Branch recovery must
        // not clear this lane globally because an older paired instruction may
        // already be in MEM/WB when a younger branch resolves in ID.
        if (external_flush) begin
            es_valid <= 1'b0;
        end
        else if (main_es_allowin) begin
            es_valid <= issue_valid;
        end

        if (main_es_allowin) begin
            es_alu_op <= issue_alu_op;
            es_src1   <= issue_src1;
            es_src2   <= issue_src2;
            es_src1_from_main_ms <= issue_src1_from_main_ms;
            es_src2_from_main_ms <= issue_src2_from_main_ms;
            es_src1_from_main_ws <= issue_src1_from_main_ws;
            es_src2_from_main_ws <= issue_src2_from_main_ws;
            es_src1_from_side_ms <= issue_src1_from_side_ms;
            es_src2_from_side_ms <= issue_src2_from_side_ms;
            es_dest_r <= issue_dest;
            es_pc     <= issue_pc;
            es_inst   <= issue_inst;
            es_is_older <= issue_is_older;
        end
        else if (es_valid) begin
            // If this Slot-1 uop is held behind a multi-cycle Slot-0 operation,
            // its producer leaves MEM before the uop can advance.  Capture the
            // registered forwarded value on the first held cycle and then use
            // the local operand register for all subsequent cycles.
            if (es_src1_from_main_ms) begin
                es_src1 <= main_ms_result;
                es_src1_from_main_ms <= 1'b0;
            end
            else if (es_src1_from_main_ws) begin
                es_src1 <= main_ws_result;
                es_src1_from_main_ws <= 1'b0;
            end
            else if (es_src1_from_side_ms) begin
                es_src1 <= ms_result_r;
                es_src1_from_side_ms <= 1'b0;
            end
            if (es_src2_from_main_ms) begin
                es_src2 <= main_ms_result;
                es_src2_from_main_ms <= 1'b0;
            end
            else if (es_src2_from_main_ws) begin
                es_src2 <= main_ws_result;
                es_src2_from_main_ws <= 1'b0;
            end
            else if (es_src2_from_side_ms) begin
                es_src2 <= ms_result_r;
                es_src2_from_side_ms <= 1'b0;
            end
        end

        if (main_ms_allowin) begin
            ms_valid <= es_valid && main_es_to_ms_valid;
            if (es_valid && main_es_to_ms_valid) begin
                ms_result_r <= es_result;
                ms_dest_r   <= es_dest_r;
                ms_pc       <= es_pc;
                ms_inst     <= es_inst;
                ms_is_older <= es_is_older;
            end
        end

        if (main_ws_allowin) begin
            ws_valid <= ms_valid && main_ms_to_ws_valid;
            if (ms_valid && main_ms_to_ws_valid) begin
                ws_result_r <= ms_result_r;
                ws_dest_r   <= ms_dest_r;
                ws_pc       <= ms_pc;
                ws_inst     <= ms_inst;
                ws_is_older <= ms_is_older;
            end
        end
    end
end

`ifdef DIFFTEST_EN
    // Match the original WB packet timing: delay one cycle so GRegState already
    // contains both same-cycle architectural writes. Slot 0 is index 0 and the
    // younger Slot 1 instruction is index 1.
    reg        dt_valid;
    reg [31:0] dt_pc;
    reg [31:0] dt_inst;
    reg [ 4:0] dt_dest;
    reg [31:0] dt_data;
    reg        dt_is_older;
    always @(posedge clk) begin
        if (reset) begin
            dt_valid <= 1'b0;
            dt_pc    <= 32'b0;
            dt_inst  <= 32'b0;
            dt_dest  <= 5'b0;
            dt_data  <= 32'b0;
            dt_is_older <= 1'b0;
        end
        else begin
            dt_valid <= ws_valid;
            dt_pc    <= ws_pc;
            dt_inst  <= ws_inst;
            dt_dest  <= ws_dest_r;
            dt_data  <= ws_result_r;
            dt_is_older <= ws_is_older;
        end
    end

    DifftestInstrCommit u_difftest_instr_commit_slot1 (
        .clock          (clk),
        .coreid         (8'd0),
        .index          (dt_is_older ? 8'd0 : 8'd1),
        .valid          (dt_valid),
        .pc             ({32'b0, dt_pc}),
        .instr          (dt_inst),
        .skip           (1'b0),
        .is_TLBFILL     (1'b0),
        .TLBFILL_index  (5'b0),
        .is_CNTinst     (1'b0),
        .timer_64_value (64'b0),
        .wen            (dt_valid),
        .wdest          ({3'b0, dt_dest}),
        .wdata          ({32'b0, dt_data}),
        .csr_rstat      (1'b0),
        .csr_data       (32'b0)
    );
`endif

endmodule


module exe_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ms_allowin    ,
    output                         es_allowin    ,
    input                          external_flush,
    //from ds
    input                          ds_to_es_valid,
    input  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to ms
    output                         es_to_ms_valid,
    output [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    //to ds: RAW hazard information
    output [ 4:0]                  es_to_ds_dest ,
    output                         es_to_ds_load_op,
    output                         es_to_ds_mem_load_op,
    output [31:0]                  es_to_ds_result,
    output                         es_stage_valid,
    // M17.8B deferred load-dependent conditional branch resolution
    output                         deferred_bp_update_en,
    output [31:0]                  deferred_bp_update_pc,
    output                         deferred_bp_update_taken,
    output [31:0]                  deferred_bp_update_target,
    output                         deferred_bp_update_is_cond,
    output                         deferred_bp_update_is_call,
    output                         deferred_bp_update_is_return,
    output                         deferred_bp_update_is_indirect,
    output                         deferred_redirect_valid,
    output                         deferred_actual_taken,
    output [31:0]                  deferred_taken_target,
    output [31:0]                  deferred_fallthrough,
    // Forwarding sources.  Destination buses are already zero when their stage
    // is invalid or not writing GPR, so EXE can use them directly.
    input  [ 4:0]                  ms_to_ds_dest,
    input                          ms_to_ds_load_op,
    input                          ms_to_es_load_ready,
    input  [31:0]                  ms_to_es_load_result,
    input  [31:0]                  ms_to_ds_result,
    input  [ 4:0]                  ws_to_ds_dest,
    input  [31:0]                  ws_to_ds_result,

    // data sram-like interface(write/read request)
    output        data_sram_req    ,
    output        data_sram_wr     ,
    output [ 1:0] data_sram_size   ,
    output [ 3:0] data_sram_wstrb  ,
    output [31:0] data_sram_addr   ,
    output [31:0] data_sram_addr_fast,
    output        data_sram_addr_late,
    output [31:0] data_sram_wdata  ,
    output        data_sram_uncached,
    input         data_sram_addr_ok,
    // CACOP sideband
    output        cacop_req,
    output [ 4:0] cacop_code,
    output [31:0] cacop_addr,
    output [31:0] cacop_paddr,
    input         cacop_addr_ok,
    input         llbit_state
);

reg         es_valid      ;
wire        es_ready_go   ;

reg  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus_r;

wire        es_inst_cacop;
wire        es_direct_addr_bypass;
wire [ 4:0] es_cacop_code;
wire [31:0] es_mem_addr_phy;
wire [ 6:0] muldiv_op   ;
wire        es_res_from_csr;
wire        es_data_uncached;
wire [31:0] es_csr_result;
wire [11:0] alu_op      ;
wire [ 7:0] es_mem_op;
wire        src1_is_pc;
wire        src2_is_imm;
wire        src2_is_4;
wire        res_from_mem;
wire        dst_is_r1;
wire        gr_we;
wire        es_mem_we;
wire [4: 0] dest;
wire [31:0] rj_value;
wire [31:0] rkd_value;
wire [31:0] imm;
wire        es_rj_from_ms;
wire        es_rk_from_ms;
wire        es_rd_from_ms;
wire        es_rj_from_ws;
wire        es_rk_from_ws;
wire        es_rd_from_ws;
wire [31:0] es_pc;
// LLSC_STAGE62B_DEC_DONE
wire [31:0] es_inst;
wire        es_main_is_younger;
wire        es_deferred_branch;
wire        es_deferred_rj_from_load;
wire        es_deferred_rd_from_load;
wire        es_deferred_pred_taken;
wire [31:0] es_deferred_pred_nextpc_r;
wire        es_dmw_addr_bypass;
wire        es_inst_ll_w;
wire        es_inst_sc_w;
wire        es_sc_success;


assign {es_main_is_younger,
        es_inst,
        es_deferred_branch,
        es_deferred_rj_from_load,
        es_deferred_rd_from_load,
        es_deferred_pred_taken,
        es_deferred_pred_nextpc_r,
        es_data_uncached,
        es_direct_addr_bypass,
        es_dmw_addr_bypass,
        es_inst_cacop,
        es_cacop_code,
        es_mem_addr_phy,
        muldiv_op,
        es_res_from_csr,
        es_csr_result,
        alu_op,
        es_mem_op,
        src1_is_pc,
        src2_is_imm,
        src2_is_4,
        gr_we,
        es_mem_we,
        dest,
        imm,
        es_rj_from_ms,
        es_rk_from_ms,
        es_rd_from_ms,
        es_rj_from_ws,
        es_rk_from_ws,
        es_rd_from_ws,
        rj_value,
        rkd_value,
        es_pc,
        res_from_mem
       } = ds_to_es_bus_r;

assign es_inst_ll_w = (es_inst[31:24] == 8'h20);
assign es_inst_sc_w = (es_inst[31:24] == 8'h21);
assign es_sc_success = es_inst_sc_w && llbit_state;

wire [31:0] alu_src1   ;
wire [31:0] alu_src2   ;
wire [31:0] alu_result ;
wire        es_mem_access;
wire [31:0] es_addr_result;
// Dedicated effective-address adder.  Loads/stores/CACOP always perform rj+imm;
// routing their Cache address through the general one-hot ALU result mux added
// several LUT levels to the current critical path.  Keep a separate carry-chain
// result for the memory path while ordinary ALU instructions retain u_alu.
(* keep = "true" *) wire [31:0] es_mem_addr_add_result;
(* keep = "true" *) wire [31:0] es_mem_addr_add_fast;

wire [31:0] rj_value_forwarded;
wire [31:0] rk_value_forwarded;
wire [31:0] rd_value_forwarded;
wire [31:0] rj_value_final;
wire [31:0] rk_value_final;
wire [31:0] rd_value_final;
wire [31:0] deferred_rj_value;
wire [31:0] deferred_rd_value;
wire        deferred_rj_eq_rd;
wire        deferred_rj_lt_rd_signed;
wire        deferred_rj_lt_rd_unsigned;
wire        deferred_taken_comb;
wire        deferred_mispredict_comb;
wire        deferred_taken_resolved;
wire        deferred_mispredict_resolved;
wire [31:0] deferred_taken_target_resolved;
wire        deferred_pred_target_miss_resolved;
wire        deferred_resolve_fire;
wire        es_normal_ready_go;
wire        es_stage_advance;
wire        es_slot_available;
reg         es_operand_hold_valid;
reg  [31:0] es_rj_value_hold;
reg  [31:0] es_rk_value_hold;
reg  [31:0] es_rd_value_hold;
reg         es_deferred_load_decision_valid;
reg         es_deferred_load_taken_r;
reg         es_deferred_load_mispredict_r;
reg  [31:0] es_deferred_load_taken_target_r;

wire        inst_muldiv;
wire        inst_div_op;
wire        inst_mod_op;
wire        inst_div_signed;
wire        inst_mul_op;
wire signed [32:0] mul_operand_a_ext;
wire signed [32:0] mul_operand_b_ext;
(* use_dsp = "yes" *) wire signed [65:0] mul_product_ext_comb;
reg  [ 2:0]        mul_product_op_r;
reg  [63:0]        mul_product_r;
reg                mul_product_valid;
wire [63:0]        mul_product_comb;
wire [31:0]        mul_result_fast;
wire [31:0]        div_result;
wire [31:0]        muldiv_result;
wire [31:0]        es_final_result;

assign inst_muldiv      = |muldiv_op;
assign inst_mul_op      = |muldiv_op[2:0];       // mul.w/mulh.w/mulh.wu
assign inst_div_op      = |muldiv_op[6:3];       // div.w/mod.w/div.wu/mod.wu
assign inst_mod_op      =  muldiv_op[4] | muldiv_op[6];
assign inst_div_signed  =  muldiv_op[3] | muldiv_op[4];

// Round 12 multiplier pipeline:
// ds_to_es_bus_r, MEM and WB forwarding sources are all registered boundaries.
// Feed those resolved EXE operands directly into one 33x33 signed DSP product
// and capture the product after one EXE cycle.  The extra sign bit makes the
// same multiplier exact for both signed and unsigned 32-bit operations, avoiding
// the duplicated signed/unsigned multiplier trees used in Round 11.
assign mul_operand_a_ext = muldiv_op[2] ? $signed({1'b0, rj_value_forwarded}) :
                                         $signed({rj_value_forwarded[31], rj_value_forwarded});
assign mul_operand_b_ext = muldiv_op[2] ? $signed({1'b0, rk_value_forwarded}) :
                                         $signed({rk_value_forwarded[31], rk_value_forwarded});
assign mul_product_ext_comb = mul_operand_a_ext * mul_operand_b_ext;
assign mul_product_comb = mul_product_ext_comb[63:0];

always @(posedge clk) begin
    if (reset || external_flush) begin
        mul_product_op_r  <= 3'b0;
        mul_product_r     <= 64'b0;
        mul_product_valid <= 1'b0;
    end
    else begin
        // The only hold enable is the local product-valid bit; unlike Round 10,
        // no ID/MMU/exception control reaches the DSP output bank.  Once the
        // product is valid it remains stable under MEM backpressure, allowing
        // the DSP input to use the shorter live forwarding mux rather than the
        // additional EXE operand-hold mux.
        if (!mul_product_valid) begin
            mul_product_op_r <= muldiv_op[2:0];
            mul_product_r    <= mul_product_comb;
        end

        if (es_allowin)
            mul_product_valid <= 1'b0;
        else if (es_valid && inst_mul_op && !mul_product_valid)
            // Product captured on this edge; it is available in the next cycle.
            mul_product_valid <= 1'b1;
    end
end

// Product_r is the timing-isolation register at the DSP output.  Select the
// architectural 32-bit half directly from it.
assign mul_result_fast = mul_product_op_r[0] ? mul_product_r[31:0] :
                                                 mul_product_r[63:32];

// ----------------------------------------------------------------------
// Multi-cycle divider for div.w/mod.w/div.wu/mod.wu.
// The old code used combinational / and %, which passed behavioral sim but
// created a >100ns EXE-to-IF timing path on FPGA.  This divider uses a simple
// 32-cycle restoring algorithm.  While it is busy, EXE is held and ID treats
// this instruction like a load-use hazard, so dependent branches cannot use an
// unfinished forwarded result.
// ----------------------------------------------------------------------
reg        div_busy;
reg        div_done;
reg        div_signed_r;
reg        div_mod_r;
reg        div_quot_neg_r;
reg        div_rem_neg_r;
reg [5:0]  div_cnt;
reg [31:0] div_dividend;
reg [31:0] div_divisor;
reg [31:0] div_quotient;
reg [32:0] div_remainder;
reg [31:0] div_result_r;

wire       div_start;
wire       div_by_zero;
wire       div_overflow;
wire       div_src1_neg;
wire       div_src2_neg;
wire [31:0] div_src1_abs;
wire [31:0] div_src2_abs;
wire [32:0] div_rem_shift;
wire [32:0] div_rem_sub;
wire [32:0] div_rem_next;
wire [31:0] div_quot_next;
wire [31:0] div_quot_final;
wire [31:0] div_rem_final;

assign div_start    = es_valid && inst_div_op && !div_busy && !div_done;
assign div_by_zero  = (rk_value_final == 32'b0);
assign div_overflow = inst_div_signed && (rj_value_final == 32'h8000_0000) && (rk_value_final == 32'hffff_ffff);
assign div_src1_neg = inst_div_signed && rj_value_final[31];
assign div_src2_neg = inst_div_signed && rk_value_final[31];
assign div_src1_abs = div_src1_neg ? (~rj_value_final + 32'b1) : rj_value_final;
assign div_src2_abs = div_src2_neg ? (~rk_value_final + 32'b1) : rk_value_final;

assign div_rem_shift = {div_remainder[31:0], div_dividend[31]};
assign div_rem_sub   = div_rem_shift - {1'b0, div_divisor};
// Use the subtraction sign as the comparison result.  The previous form
// described both a 33-bit comparator and a 33-bit subtractor; although Vivado
// can sometimes share them, making the sharing explicit removes a duplicated
// arithmetic cone from the divider feedback path.
wire div_sub_nonnegative = ~div_rem_sub[32];
assign div_rem_next  = div_sub_nonnegative ? div_rem_sub : div_rem_shift;
assign div_quot_next = {div_quotient[30:0], div_sub_nonnegative};
assign div_quot_final = div_quot_neg_r ? (~div_quot_next + 32'b1) : div_quot_next;
assign div_rem_final  = div_rem_neg_r  ? (~div_rem_next[31:0] + 32'b1) : div_rem_next[31:0];
assign div_result     = div_result_r;

always @(posedge clk) begin
    if (reset) begin
        div_busy      <= 1'b0;
        div_done      <= 1'b0;
        div_signed_r  <= 1'b0;
        div_mod_r     <= 1'b0;
        div_quot_neg_r<= 1'b0;
        div_rem_neg_r <= 1'b0;
        div_cnt       <= 6'b0;
        div_dividend  <= 32'b0;
        div_divisor   <= 32'b0;
        div_quotient  <= 32'b0;
        div_remainder <= 33'b0;
        div_result_r  <= 32'b0;
    end
    else begin
        // Once the current EXE instruction is allowed to leave, clear the
        // divider completion flag so the next div/mod instruction can start.
        if (es_allowin) begin
            div_done <= 1'b0;
        end

        if (div_start) begin
            if (div_by_zero) begin
                div_busy     <= 1'b0;
                div_done     <= 1'b1;
                div_result_r <= 32'b0;
            end
            else if (div_overflow) begin
                div_busy     <= 1'b0;
                div_done     <= 1'b1;
                div_result_r <= inst_mod_op ? 32'b0 : 32'h8000_0000;
            end
            else begin
                div_busy       <= 1'b1;
                div_done       <= 1'b0;
                div_signed_r   <= inst_div_signed;
                div_mod_r      <= inst_mod_op;
                div_quot_neg_r <= div_src1_neg ^ div_src2_neg;
                div_rem_neg_r  <= div_src1_neg;
                div_cnt        <= 6'b0;
                div_dividend   <= div_src1_abs;
                div_divisor    <= div_src2_abs;
                div_quotient   <= 32'b0;
                div_remainder  <= 33'b0;
            end
        end
        else if (div_busy) begin
            div_dividend  <= {div_dividend[30:0], 1'b0};
            div_quotient  <= div_quot_next;
            div_remainder <= div_rem_next;

            if (div_cnt == 6'd31) begin
                div_busy     <= 1'b0;
                div_done     <= 1'b1;
                div_result_r <= div_mod_r ? div_rem_final : div_quot_final;
            end
            else begin
                div_cnt <= div_cnt + 6'b1;
            end
        end
    end
end

assign muldiv_result = ({32{inst_mul_op}} & mul_result_fast) |
                       ({32{inst_div_op}} & div_result);

// LLSC_STAGE62B_PATH_DONE
assign es_mem_access = ((|es_mem_op) && (!es_inst_sc_w || es_sc_success)) | es_inst_cacop;
assign es_mem_addr_add_result = rj_value_final + imm;
assign es_mem_addr_add_fast = rj_value + imm;
assign es_addr_result = es_mem_access ?
                        (es_direct_addr_bypass ? es_mem_addr_add_result :
                         es_dmw_addr_bypass ?
                             {es_mem_addr_phy[31:29], es_mem_addr_add_result[28:0]} :
                             es_mem_addr_phy) :
                        alu_result;

assign es_final_result = es_inst_sc_w ? (es_sc_success ? 32'd1 : 32'd0) :
                         es_res_from_csr ? es_csr_result :
                         inst_muldiv     ? muldiv_result : es_addr_result;


// did't use in lab7
wire        es_res_from_mem;
assign es_res_from_mem = |es_mem_op[4:0];

// Send valid destination register number back to ID for blocking.
// Only real register-write instructions should be considered hazards.
assign es_to_ds_dest    = dest & {5{es_valid && gr_we}};
// A multiply is non-forwardable only until its registered product is valid.
// Once mul_product_valid=1, a dependent instruction may enter EXE on the same
// edge that the multiply advances to MEM and consume the registered MEM result
// through the predecoded next-MEM selector.
assign es_to_ds_load_op = ((|es_mem_op[4:0]) ||
                           (inst_mul_op && !mul_product_valid) ||
                           (inst_div_op && !div_done)) && es_valid;
assign es_to_ds_mem_load_op = (|es_mem_op[4:0]) && es_valid;
assign es_to_ds_result  = inst_mul_op ? 32'b0 : es_final_result;
assign es_stage_valid   = es_valid;


assign es_to_ms_bus = {es_main_is_younger,
                         es_inst,          // 32, instruction for Difftest
                         es_inst_cacop,  //76:76, CACOP operation, used to flush younger instructions after ICache maintenance
                       es_mem_access,  //75:75, load/store/CACOP request waits for data_ok in MEM
                       es_mem_op[4:0], //74:70, load type: ld.b/ld.h/ld.w/ld.bu/ld.hu
                       gr_we       ,  //69:69 1
                       dest        ,  //68:64 5
                       es_final_result,  //63:32 32, also keeps load address for MEM
                       es_pc          //31:0  32
                      };

// For the SRAM-like data bus, do not issue a memory request until MEM can
// accept this instruction.  Otherwise addr_ok may be consumed while the EXE
// stage is still blocked by MEM, and the later data_ok will be associated
// with the wrong instruction under random-delay verification.
assign es_normal_ready_go = (!inst_mul_op || mul_product_valid) &&
                            (!inst_div_op || div_done) &&
                            (!es_mem_access ||
                             (ms_allowin && (es_inst_cacop ? cacop_addr_ok :
                                                                  data_sram_addr_ok)));
// A deferred branch waits in EXE until the producer load has a complete,
// formatted MEM result.  This is the only cache-data bypass into the branch
// comparator; no bit of ms_to_es_load_result is visible in ID.
wire es_deferred_wait_load = es_deferred_rj_from_load |
                             es_deferred_rd_from_load;
assign es_ready_go = es_deferred_branch ?
                     (!es_deferred_wait_load ||
                      es_deferred_load_decision_valid) :
                     es_normal_ready_go;
assign es_stage_advance = es_valid && es_ready_go && ms_allowin;
assign deferred_resolve_fire = es_stage_advance && es_deferred_branch && !external_flush;
assign es_slot_available = !es_valid || es_stage_advance;
// A correctly predicted control transfer remains a normal one-cycle EXE
// instruction and accepts the following ID instruction without a bubble.
// A deferred misprediction must withhold the younger instruction on its
// resolve edge: ordinary branch recovery flushes IF/ID, while EXE's
// external_flush input is intentionally reserved for CACOP.
assign es_allowin     = es_slot_available &&
                        !(deferred_resolve_fire && deferred_mispredict_resolved);
assign es_to_ms_valid = es_valid && es_ready_go && !external_flush;
always @(posedge clk) begin
    if (reset) begin
        es_valid <= 1'b0;
    end
    else if (external_flush) begin
        es_valid <= 1'b0;
    end
    else if (deferred_resolve_fire) begin
        // The branch itself advances to MEM.  A correctly predicted branch has
        // es_allowin=1 and replaces itself with the accepted younger ID
        // instruction; a misprediction has es_allowin=0 and leaves EXE empty
        // until the registered redirect flush arrives.
        es_valid <= es_allowin && ds_to_es_valid;
    end
    else if (es_allowin) begin
        es_valid <= ds_to_es_valid;
    end

    // exp19 ref-opt v5:
    // Do not gate the wide EXE bus/DSP input registers with ds_to_es_valid.
    // ds_to_es_valid contains ID exception/TLB logic and was driving the DSP48
    // internal clock-enables.  It is safe to load don't-care bus data when
    // es_valid is 0; all side effects are already guarded by es_valid.
    if (es_allowin) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
end

// ----------------------------------------------------------------------
// MEM/WB -> EXE operand forwarding.
// This removes the biggest CPI loss of the simple five-stage pipeline: a
// blanket one-cycle stall for every ALU RAW dependency.  Load data is still
// not forwarded from MEM because ms_to_ds_result intentionally carries only
// non-load results; load-use waits until WB.
// ----------------------------------------------------------------------
// Round 10 forwarding: ID has already decided which future stage owns each
// operand.  EXE therefore uses one-bit selects rather than comparing three
// register numbers against MEM/WB destinations on the Cache-address path.
assign rj_value_forwarded = es_rj_from_ms ? ms_to_ds_result :
                            es_rj_from_ws ? ws_to_ds_result : rj_value;
assign rk_value_forwarded = es_rk_from_ms ? ms_to_ds_result :
                            es_rk_from_ws ? ws_to_ds_result : rkd_value;
assign rd_value_forwarded = es_rd_from_ms ? ms_to_ds_result :
                            es_rd_from_ws ? ws_to_ds_result : rkd_value;

// Preserve transient MEM/WB forwarding values whenever the current EXE
// instruction is held.  The architectural operands are fixed once an in-order
// instruction reaches EXE, so capturing them on the first blocked cycle is
// equivalent to keeping the forwarding mux continuously visible, but remains
// correct after the producer leaves WB.
assign rj_value_final = es_operand_hold_valid ? es_rj_value_hold : rj_value_forwarded;
assign rk_value_final = es_operand_hold_valid ? es_rk_value_hold : rk_value_forwarded;
assign rd_value_final = es_operand_hold_valid ? es_rd_value_hold : rd_value_forwarded;

// M17.8B EXE-side conditional-branch comparator.  Ordinary controls use only
// registered/held EXE operands here.  A load-dependent control is resolved by
// the dedicated capture comparator below and selects its registered decision,
// so no MEM load-value bit can feed this ordinary compare cone.
// Experimental ready-MEM-load branch release.
//
// A branch released alongside a ready MEM load consumes the registered WB
// value in its first EXE cycle.  This path is used only by the predecoded
// es_*_from_ws selects; the load value never enters the ID redirect cone.
assign deferred_rj_value = es_operand_hold_valid ? es_rj_value_hold :
                           es_rj_from_ms ? ms_to_ds_result :
                           es_rj_from_ws ? ws_to_ds_result : rj_value;
assign deferred_rd_value = es_operand_hold_valid ? es_rd_value_hold :
                           es_rd_from_ms ? ms_to_ds_result :
                           es_rd_from_ws ? ws_to_ds_result : rkd_value;
assign deferred_rj_eq_rd = (deferred_rj_value == deferred_rd_value);
assign deferred_rj_lt_rd_signed = ($signed(deferred_rj_value) <
                                   $signed(deferred_rd_value));
assign deferred_rj_lt_rd_unsigned = (deferred_rj_value < deferred_rd_value);
assign deferred_taken_comb =
       ((es_inst[31:26] == 6'h16) &&  deferred_rj_eq_rd) |
       ((es_inst[31:26] == 6'h17) && !deferred_rj_eq_rd) |
       ((es_inst[31:26] == 6'h18) &&  deferred_rj_lt_rd_signed) |
       ((es_inst[31:26] == 6'h19) && !deferred_rj_lt_rd_signed) |
       ((es_inst[31:26] == 6'h1a) &&  deferred_rj_lt_rd_unsigned) |
       ((es_inst[31:26] == 6'h1b) && !deferred_rj_lt_rd_unsigned) |
       (es_inst[31:26] == 6'h13);
wire [31:0] deferred_branch_offs =
       {{14{es_inst[25]}}, es_inst[25:10], 2'b0};
wire deferred_is_jirl = (es_inst[31:26] == 6'h13);
wire [31:0] deferred_taken_target_comb =
       deferred_is_jirl ?
       (deferred_rj_value + {{14{es_inst[25]}}, es_inst[25:10], 2'b0}) :
       (es_pc + deferred_branch_offs);
wire deferred_pred_target_miss =
       es_deferred_pred_nextpc_r != deferred_taken_target_comb;
assign deferred_mispredict_comb =
       (es_deferred_pred_taken ^ deferred_taken_comb) |
       (deferred_taken_comb && es_deferred_pred_taken &&
        deferred_pred_target_miss);

// M19.12 load-branch decision boundary.  M19.11 registered the formatted load
// value and compared it one cycle later, leaving value-register -> 32-bit
// compare -> global allow-in as 63 of the 100 worst paths.  Compute the same
// decision on the MEM-result capture edge and register only the compact
// taken/mispredict/target result.  The branch still resolves on the following
// cycle, exactly as in M19.11, but the global recovery network now starts at a
// local EXE register rather than at the comparator carry chain.
wire [31:0] deferred_load_capture_rj = es_deferred_rj_from_load ?
                                           ms_to_es_load_result : rj_value_final;
wire [31:0] deferred_load_capture_rd = es_deferred_rd_from_load ?
                                           ms_to_es_load_result : rd_value_final;
wire deferred_load_capture_eq =
       (deferred_load_capture_rj == deferred_load_capture_rd);
wire deferred_load_capture_lt_signed =
       ($signed(deferred_load_capture_rj) <
        $signed(deferred_load_capture_rd));
wire deferred_load_capture_lt_unsigned =
       (deferred_load_capture_rj < deferred_load_capture_rd);
wire deferred_load_capture_taken =
       ((es_inst[31:26] == 6'h16) &&  deferred_load_capture_eq) |
       ((es_inst[31:26] == 6'h17) && !deferred_load_capture_eq) |
       ((es_inst[31:26] == 6'h18) &&  deferred_load_capture_lt_signed) |
       ((es_inst[31:26] == 6'h19) && !deferred_load_capture_lt_signed) |
       ((es_inst[31:26] == 6'h1a) &&  deferred_load_capture_lt_unsigned) |
       ((es_inst[31:26] == 6'h1b) && !deferred_load_capture_lt_unsigned) |
       deferred_is_jirl;
wire [31:0] deferred_load_capture_target = deferred_is_jirl ?
       (deferred_load_capture_rj +
        {{14{es_inst[25]}}, es_inst[25:10], 2'b0}) :
       (es_pc + deferred_branch_offs);
wire deferred_load_capture_target_miss =
       es_deferred_pred_nextpc_r != deferred_load_capture_target;
wire deferred_load_capture_mispredict =
       (es_deferred_pred_taken ^ deferred_load_capture_taken) |
       (deferred_load_capture_taken && es_deferred_pred_taken &&
        deferred_load_capture_target_miss);

assign deferred_taken_resolved = es_deferred_wait_load ?
                                  es_deferred_load_taken_r :
                                  deferred_taken_comb;
assign deferred_mispredict_resolved = es_deferred_wait_load ?
                                       es_deferred_load_mispredict_r :
                                       deferred_mispredict_comb;
assign deferred_taken_target_resolved = es_deferred_wait_load ?
                                         es_deferred_load_taken_target_r :
                                         deferred_taken_target_comb;
assign deferred_pred_target_miss_resolved =
       es_deferred_pred_nextpc_r != deferred_taken_target_resolved;

assign deferred_bp_update_en     = deferred_resolve_fire;
assign deferred_bp_update_pc     = es_pc;
assign deferred_bp_update_taken  = deferred_taken_resolved;
assign deferred_bp_update_target = deferred_taken_target_resolved;
assign deferred_bp_update_is_cond = !deferred_is_jirl;
assign deferred_bp_update_is_call = deferred_is_jirl && (dest == 5'd1);
assign deferred_bp_update_is_return = deferred_is_jirl &&
                                      (dest == 5'd0) &&
                                      (es_inst[9:5] == 5'd1) &&
                                      (es_inst[25:10] == 16'b0);
assign deferred_bp_update_is_indirect = deferred_is_jirl &&
                                        !deferred_bp_update_is_return;
assign deferred_redirect_valid   = deferred_resolve_fire &&
                                    deferred_mispredict_resolved;
assign deferred_actual_taken     = deferred_taken_resolved;
assign deferred_taken_target     = deferred_taken_target_resolved;
assign deferred_fallthrough      = es_pc + 32'd4;

always @(posedge clk) begin
    if (reset || external_flush) begin
        es_operand_hold_valid <= 1'b0;
        es_rj_value_hold      <= 32'b0;
        es_rk_value_hold      <= 32'b0;
        es_rd_value_hold      <= 32'b0;
    end
    else if (es_allowin || es_stage_advance) begin
        // The current instruction leaves (or EXE is empty).  The next accepted
        // instruction starts with live forwarding rather than stale hold data.
        es_operand_hold_valid <= 1'b0;
    end
    else if (es_valid && !es_operand_hold_valid) begin
        es_operand_hold_valid <= 1'b1;
        es_rj_value_hold      <= rj_value_forwarded;
        es_rk_value_hold      <= rk_value_forwarded;
        es_rd_value_hold      <= rd_value_forwarded;
    end
end

always @(posedge clk) begin
    if (reset || external_flush) begin
        es_deferred_load_decision_valid <= 1'b0;
        es_deferred_load_taken_r         <= 1'b0;
        es_deferred_load_mispredict_r    <= 1'b0;
        es_deferred_load_taken_target_r  <= 32'b0;
    end
    else if (es_allowin) begin
        es_deferred_load_decision_valid <= 1'b0;
    end
    else if (es_valid && es_deferred_branch &&
             es_deferred_wait_load && ms_to_es_load_ready &&
             !es_deferred_load_decision_valid) begin
        es_deferred_load_decision_valid <= 1'b1;
        es_deferred_load_taken_r        <= deferred_load_capture_taken;
        es_deferred_load_mispredict_r   <= deferred_load_capture_mispredict;
        es_deferred_load_taken_target_r <= deferred_load_capture_target;
    end
end

assign alu_src1 = src1_is_pc  ? es_pc : rj_value_final;
assign alu_src2 = src2_is_imm ? imm   : rk_value_final;

alu u_alu(
    .alu_op     (alu_op    ),
    .alu_src1   (alu_src1  ),
    .alu_src2   (alu_src2  ),
    .alu_result (alu_result)
    );

wire [3:0] st_b_we;
wire [3:0] st_h_we;
wire [3:0] st_w_we;
wire [31:0] st_b_wdata;
wire [31:0] st_h_wdata;
wire [31:0] st_w_wdata;

assign st_b_we = (es_addr_result[1:0] == 2'b00) ? 4'b0001 :
                 (es_addr_result[1:0] == 2'b01) ? 4'b0010 :
                 (es_addr_result[1:0] == 2'b10) ? 4'b0100 : 4'b1000;
assign st_h_we = es_addr_result[1] ? 4'b1100 : 4'b0011;
assign st_w_we = 4'b1111;

assign st_b_wdata = (es_addr_result[1:0] == 2'b00) ? {24'b0, rd_value_final[7:0]} :
                    (es_addr_result[1:0] == 2'b01) ? {16'b0, rd_value_final[7:0], 8'b0} :
                    (es_addr_result[1:0] == 2'b10) ? {8'b0,  rd_value_final[7:0], 16'b0} :
                                                      {rd_value_final[7:0], 24'b0};
assign st_h_wdata = es_addr_result[1] ? {rd_value_final[15:0], 16'b0} : {16'b0, rd_value_final[15:0]};
assign st_w_wdata = rd_value_final;

wire        es_mem_byte;
wire        es_mem_half;
wire        es_mem_word;
wire [ 3:0] data_wstrb_raw;

assign es_mem_byte = es_mem_op[0] | es_mem_op[3] | es_mem_op[5];
assign es_mem_half = es_mem_op[1] | es_mem_op[4] | es_mem_op[6];
assign es_mem_word = es_mem_op[2] | es_mem_op[7];

assign data_wstrb_raw = ({4{es_mem_op[5]}} & st_b_we) |
                        ({4{es_mem_op[6]}} & st_h_we) |
                        ({4{es_mem_op[7]}} & st_w_we);

assign data_sram_req   = es_valid && (|es_mem_op) && ms_allowin &&
                         (!es_inst_sc_w || es_sc_success);
assign cacop_req        = es_valid && es_inst_cacop && ms_allowin;
assign data_sram_wr    = data_sram_req && es_mem_we;
assign data_sram_size  = es_mem_byte ? 2'b00 :
                         es_mem_half ? 2'b01 : 2'b10;
assign data_sram_wstrb = data_sram_wr ? data_wstrb_raw : 4'h0;
assign data_sram_addr  = es_addr_result;
// The fast cache address is exact whenever no late-base flag is asserted.  TLB
// accesses already use a registered physical address and therefore never need
// the selective wrapper boundary, even if an obsolete forwarding tag remains
// in the instruction payload.
assign data_sram_addr_fast =
       es_direct_addr_bypass ? es_mem_addr_add_fast :
       es_dmw_addr_bypass    ? {es_mem_addr_phy[31:29],
                                es_mem_addr_add_fast[28:0]} :
                               es_mem_addr_phy;
assign data_sram_addr_late = (es_direct_addr_bypass || es_dmw_addr_bypass) &&
                             (es_rj_from_ms || es_rj_from_ws);
assign data_sram_wdata = ({32{es_mem_op[5]}} & st_b_wdata) |
                         ({32{es_mem_op[6]}} & st_h_wdata) |
                         ({32{es_mem_op[7]}} & st_w_wdata);
assign data_sram_uncached = data_sram_req && es_data_uncached;

assign cacop_code = es_cacop_code;
assign cacop_addr = es_addr_result;
assign cacop_paddr = es_addr_result;


endmodule



module mem_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ws_allowin    ,
    output                         ms_allowin    ,
    //from es
    input                          es_to_ms_valid,
    input  [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    //to ws
    output                         ms_to_ws_valid,
    output [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus  ,
    //to ds: RAW hazard information / forwarding result
    output [ 4:0]                  ms_to_ds_dest ,
    output                         ms_to_ds_load_op,
    output                         ms_to_ds_load_leave,
    output                         ms_to_es_load_ready,
    output [31:0]                  ms_to_es_load_result,
    output [31:0]                  ms_to_ds_result,
    output                         ms_stage_valid,
    output                         cacop_flush,
    output [31:0]                  cacop_flush_target,
    
    
    //from data-sram-like
    input                          data_sram_data_ok,
    input  [31                 :0] data_sram_rdata
);

reg         ms_valid;
wire        ms_ready_go;

reg [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus_r;
wire        ms_inst_cacop;
wire        ms_mem_access;
wire [ 4:0] ms_load_op;
wire        ms_res_from_mem;
wire        ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_alu_result;
wire [31:0] ms_pc;
wire [31:0] ms_inst;
wire        ms_main_is_younger;

wire [31:0] mem_result;
wire [31:0] ms_final_result;


assign {ms_main_is_younger,
        ms_inst,
        ms_inst_cacop  ,  //76:76, CACOP operation
        ms_mem_access  ,  //75:75, load/store/CACOP waits for data_ok
        ms_load_op     ,  //74:70, load type
        ms_gr_we       ,  //69:69
        ms_dest        ,  //68:64
        ms_alu_result  ,  //63:32
        ms_pc             //31:0
       } = es_to_ms_bus_r;

assign ms_to_ds_dest    = ms_dest & {5{ms_valid && ms_gr_we}};
assign ms_to_ds_load_op = ms_valid && ms_res_from_mem;
// A load that is ready and accepted by WB at this edge can be consumed by an
// EXE-forwardable dependent instruction entering EXE on the same edge.
assign ms_to_ds_load_leave = ms_valid && ms_res_from_mem && ms_ready_go && ws_allowin;
// M17.8B's only direct load-result consumer is the deferred EXE branch unit.
// Keep the valid qualifier identical to load_leave so the comparator never
// observes an incomplete AXI/cache response.  ID still receives ms_alu_result.
assign ms_to_es_load_ready  = ms_to_ds_load_leave;
assign ms_to_es_load_result = ms_final_result;
// ID-stage forwarding from MEM is still used only for non-load instructions.
// A released load consumer obtains the value from WB in its EXE cycle; do not
// expose mem_result/data_sram_rdata on this bypass bus.  This cuts the AXI R-channel
// rvalid/rdata -> MEM load result -> ID TLB/MMU/CSR timing path.
assign ms_to_ds_result  = ms_alu_result;
assign ms_stage_valid  = ms_valid;
assign cacop_flush        = ms_valid && ms_inst_cacop && data_sram_data_ok;
assign cacop_flush_target = ms_pc + 32'd4;

assign ms_to_ws_bus = {ms_main_is_younger,
                         ms_inst,         // 32, instruction for Difftest
                         ms_gr_we       ,  //69:69
                       ms_dest        ,  //68:64
                       ms_final_result,  //63:32
                       ms_pc             //31:0
                      };

reg         ms_data_buf_valid;
reg  [31:0] ms_data_buf;
wire        es_to_ms_mem_access;
wire        ms_old_mem_waiting;
wire        new_data_ok_for_new_req;
wire [31:0] data_sram_rdata_final;

assign es_to_ms_mem_access = es_to_ms_bus[75];
// When the current MEM-stage instruction is a memory access and has not yet
// received its data_ok, any data_sram_data_ok in this cycle belongs to this
// old instruction, not to a new instruction entering MEM in the same cycle.
assign ms_old_mem_waiting = ms_valid && ms_mem_access && !ms_data_buf_valid;
assign new_data_ok_for_new_req = es_to_ms_mem_access && data_sram_data_ok && !ms_old_mem_waiting;
assign data_sram_rdata_final = ms_data_buf_valid ? ms_data_buf : data_sram_rdata;

assign ms_ready_go    = !ms_mem_access || ms_data_buf_valid || data_sram_data_ok;
assign ms_allowin     = !ms_valid || ms_ready_go && ws_allowin;
assign ms_to_ws_valid = ms_valid && ms_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ms_valid          <= 1'b0;
        ms_data_buf_valid <= 1'b0;
        ms_data_buf       <= 32'b0;
    end
    else begin
        if (ms_allowin) begin
            ms_valid <= es_to_ms_valid;
        end

        if (es_to_ms_valid && ms_allowin) begin
            es_to_ms_bus_r <= es_to_ms_bus;
            // If data_ok returns in the same cycle that a *new* request enters
            // MEM and there is no older outstanding memory request, buffer it
            // for the new MEM-stage instruction.
            //
            // Important: when an old memory instruction is leaving MEM in this
            // same cycle, data_ok belongs to the old instruction.  Do not
            // incorrectly buffer that old response for the new instruction, or
            // the next load may consume a previous store/load response under
            // random-delay SRAM verification.
            if (new_data_ok_for_new_req) begin
                ms_data_buf_valid <= 1'b1;
                ms_data_buf       <= data_sram_rdata;
            end
            else begin
                ms_data_buf_valid <= 1'b0;
            end
        end
        else if (ms_valid && ms_mem_access && data_sram_data_ok && !ms_ready_go) begin
            ms_data_buf_valid <= 1'b1;
            ms_data_buf       <= data_sram_rdata;
        end
        else if (ms_ready_go && ws_allowin) begin
            ms_data_buf_valid <= 1'b0;
        end
    end
end

assign ms_res_from_mem = |ms_load_op;

wire [7:0]  load_byte_result;
wire [15:0] load_half_result;
assign load_byte_result = (ms_alu_result[1:0] == 2'b00) ? data_sram_rdata_final[ 7: 0] :
                          (ms_alu_result[1:0] == 2'b01) ? data_sram_rdata_final[15: 8] :
                          (ms_alu_result[1:0] == 2'b10) ? data_sram_rdata_final[23:16] :
                                                           data_sram_rdata_final[31:24];
assign load_half_result = ms_alu_result[1] ? data_sram_rdata_final[31:16] : data_sram_rdata_final[15:0];

assign mem_result = ({32{ms_load_op[0]}} & {{24{load_byte_result[7]}}, load_byte_result}) | // ld.b
                    ({32{ms_load_op[1]}} & {{16{load_half_result[15]}}, load_half_result}) | // ld.h
                    ({32{ms_load_op[2]}} & data_sram_rdata_final) |                              // ld.w
                    ({32{ms_load_op[3]}} & {24'b0, load_byte_result}) |                    // ld.bu
                    ({32{ms_load_op[4]}} & {16'b0, load_half_result});                     // ld.hu
assign ms_final_result = ms_res_from_mem ? mem_result : ms_alu_result;

endmodule



module wb_stage(
    input                           clk           ,
    input                           reset         ,
    //allowin
    output                          ws_allowin    ,
    //from ms
    input                           ms_to_ws_valid,
    input  [`MS_TO_WS_BUS_WD -1:0]  ms_to_ws_bus  ,
    //to rf: for write back
    output [`WS_TO_RF_BUS_WD -1:0]  ws_to_rf_bus  ,
    //to ds: RAW hazard information / forwarding result
    output [ 4:0]                   ws_to_ds_dest ,
    output [31:0]                   ws_to_ds_result,
    output                          ws_stage_valid,
    output                          ll_w_commit,
    output                          sc_w_commit,
    //trace debug interface
    output [31:0] debug_wb_pc     ,
    output [ 3:0] debug_wb_rf_we  ,
    output [ 4:0] debug_wb_rf_wnum,
    output [31:0] debug_wb_rf_wdata
);

reg         ws_valid;
wire        ws_ready_go;

reg [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus_r;
wire        ws_gr_we;
wire [ 4:0] ws_dest;
wire [31:0] ws_final_result;
wire [31:0] ws_pc;
wire [31:0] ws_inst;
wire        ws_main_is_younger;
assign {ws_main_is_younger,
        ws_inst,
        ws_gr_we       ,  //69:69
        ws_dest        ,  //68:64
        ws_final_result,  //63:32
        ws_pc             //31:0
       } = ms_to_ws_bus_r;

assign ws_to_ds_dest   = ws_dest & {5{ws_valid && ws_gr_we}};
assign ws_to_ds_result = ws_final_result;
assign ws_stage_valid  = ws_valid;
assign ll_w_commit = ws_valid && (ws_inst[31:24] == 8'h20);
assign sc_w_commit = ws_valid && (ws_inst[31:24] == 8'h21);

wire        rf_we;
wire [4 :0] rf_waddr;
wire [31:0] rf_wdata;
assign ws_to_rf_bus = {rf_we   ,  //37:37
                       rf_waddr,  //36:32
                       rf_wdata   //31:0
                      };

assign ws_ready_go = 1'b1;
assign ws_allowin  = !ws_valid || ws_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ws_valid <= 1'b0;
    end
    else if (ws_allowin) begin
        ws_valid <= ms_to_ws_valid;
    end

    if (ms_to_ws_valid && ws_allowin) begin
        ms_to_ws_bus_r <= ms_to_ws_bus;
    end
end

assign rf_we    = ws_gr_we && ws_valid;
assign rf_waddr = ws_dest;
assign rf_wdata = ws_final_result;

// debug info generate
assign debug_wb_pc       = ws_pc;
assign debug_wb_rf_we    = {4{rf_we}};
assign debug_wb_rf_wnum  = ws_dest;
assign debug_wb_rf_wdata = ws_final_result;



`ifdef DIFFTEST_EN
    // Difftest sees the architectural state after the committed instruction.
    // The commit packet is delayed by one clock so the GPR write at WB has
    // completed before the DPI callback observes GRegState/CSRRegState.
    reg        dt_commit_valid;
    reg [31:0] dt_commit_pc;
    reg [31:0] dt_commit_inst;
    reg        dt_commit_wen;
    reg [4:0]  dt_commit_wdest;
    reg [31:0] dt_commit_wdata;
    reg        dt_commit_is_younger;

    // dt_eret_instrcommit_v3:
    // ERTN    sys_stage  ?         WB  ? Difftest     ?   ?    
    reg        dt_eret_commit_valid;
    reg [31:0] dt_eret_commit_pc;
    reg [31:0] dt_eret_commit_inst;
    // TLBFILL requires its architectural write index in Difftest.
    // Keep the index captured by sys_stage, before tlbfill_index increments.
    reg        dt_eret_commit_is_tlbfill;
    reg [ 4:0]  dt_eret_commit_tlbfill_index;

    always @(posedge clk) begin
        if (reset) begin
            dt_commit_valid <= 1'b0;
            dt_commit_pc    <= 32'b0;
            dt_commit_inst  <= 32'b0;
            dt_commit_wen   <= 1'b0;
            dt_commit_wdest <= 5'b0;
            dt_commit_wdata      <= 32'b0;
            dt_commit_is_younger <= 1'b0;
            dt_eret_commit_valid         <= 1'b0;
            dt_eret_commit_pc            <= 32'b0;
            dt_eret_commit_inst          <= 32'b0;
            dt_eret_commit_is_tlbfill    <= 1'b0;
            dt_eret_commit_tlbfill_index <= 5'b0;
        end
        else begin
            dt_commit_valid <= ws_valid;
            dt_commit_pc    <= ws_pc;
            dt_commit_inst  <= ws_inst;
            dt_commit_wen   <= ws_gr_we;
            dt_commit_wdest <= ws_dest;
            dt_commit_wdata      <= ws_final_result;
            dt_commit_is_younger <= ws_main_is_younger;
            // ERTN and TLB maintenance instructions commit through sys_stage rather
            // than the normal WB Difftest path. Emit a delayed packet after the
            // system-side effect has committed, so REF executes the same instruction.
            dt_eret_commit_valid <= id_stage.sys_commit_fire &&
                                    (id_stage.sys_inst_ertn_r ||
                                     id_stage.sys_inst_tlbwr_r ||
                                     id_stage.sys_inst_tlbrd_r ||
                                     id_stage.sys_inst_tlbfill_r ||
                                     id_stage.sys_inst_tlbsrch_r ||
                                     id_stage.sys_inst_invtlb_r);
            dt_eret_commit_pc            <= id_stage.sys_pc_r;
            dt_eret_commit_inst          <= id_stage.sys_inst_r;
            dt_eret_commit_is_tlbfill    <= id_stage.sys_commit_fire &&
                                            id_stage.sys_inst_tlbfill_r;
            dt_eret_commit_tlbfill_index <= id_stage.sys_tlbfill_index_r;
        end
    end

    // dt_eret_instrcommit_v3:
    //     ?       WB  ERTN ?   sys_stage   ?   Difftest  ?  
    wire        dt_instr_valid = dt_eret_commit_valid ? 1'b1
                                                       : dt_commit_valid;
    wire [31:0] dt_instr_pc    = dt_eret_commit_valid ? dt_eret_commit_pc
                                                       : dt_commit_pc;
    wire [31:0] dt_instr_inst  = dt_eret_commit_valid ? dt_eret_commit_inst
                                                       : dt_commit_inst;
    wire        dt_instr_wen   = dt_eret_commit_valid ? 1'b0
                                                       : dt_commit_wen;
    wire [4:0]  dt_instr_wdest = dt_eret_commit_valid ? 5'b0
                                                       : dt_commit_wdest;
    wire [31:0] dt_instr_wdata = dt_eret_commit_valid ? 32'b0
                                                       : dt_commit_wdata;

    // Only a delayed system commit can represent TLBFILL.
    wire       dt_is_tlbfill = dt_eret_commit_valid &&
                                dt_eret_commit_is_tlbfill;
    wire [4:0] dt_tlbfill_index = dt_eret_commit_tlbfill_index;

    // StableCounter instructions need explicit Difftest synchronization.
    // Decode follows id_stage rdcntid/rdcntvl/rdcntvh logic.
    wire        dt_cnt_base =
        (dt_instr_inst[31:26] == 6'h00) &&
        (dt_instr_inst[25:22] == 4'h0)  &&
        (dt_instr_inst[21:20] == 2'h0)  &&
        (dt_instr_inst[19:15] == 5'h00);

    wire [4:0]  dt_cnt_rk = dt_instr_inst[14:10];
    wire [4:0]  dt_cnt_rj = dt_instr_inst[ 9: 5];
    wire [4:0]  dt_cnt_rd = dt_instr_inst[ 4: 0];

    wire        dt_rdcntvl_w =
        dt_cnt_base && (dt_cnt_rk == 5'h18) && (dt_cnt_rj == 5'h00);

    wire        dt_rdcntvh_w =
        dt_cnt_base && (dt_cnt_rk == 5'h19) && (dt_cnt_rj == 5'h00);

    wire        dt_rdcntid_w =
        dt_cnt_base && (dt_cnt_rk == 5'h18) &&
        (dt_cnt_rd == 5'h00) && !dt_rdcntvl_w;

    wire        dt_is_cntinst =
        dt_rdcntvl_w || dt_rdcntvh_w || dt_rdcntid_w;

    wire [63:0] dt_timer_64_value =
        dt_rdcntvl_w ? {id_stage.stable_counter[63:32], dt_instr_wdata} :
        dt_rdcntvh_w ? {dt_instr_wdata, id_stage.stable_counter[31:0]} :
                       id_stage.stable_counter;

    DifftestInstrCommit u_difftest_instr_commit (
        .clock          (clk),
        .coreid         (8'd0),
        .index          (dt_eret_commit_valid ? 8'd0 :
                         (dt_commit_is_younger ? 8'd1 : 8'd0)),
        .valid          (dt_instr_valid),
        .pc             ({32'b0, dt_instr_pc}),
        .instr          (dt_instr_inst),
        .skip           (1'b0),
        .is_TLBFILL     (dt_is_tlbfill),
        .TLBFILL_index  (dt_tlbfill_index),
        .is_CNTinst     (dt_is_cntinst),
        .timer_64_value (dt_timer_64_value),
        .wen            (dt_instr_wen),
        .wdest          ({3'b0, dt_instr_wdest}),
        .wdata          ({32'b0, dt_instr_wdata}),
        .csr_rstat      (1'b0),
        .csr_data       (32'b0)
    );

    DifftestTrapEvent u_difftest_trap_event (
        .clock    (clk),
        .coreid   (8'd0),
        .valid    (1'b0),
        .code     (3'b0),
        .pc       (64'b0),
        .cycleCnt (64'b0),
        .instrCnt (64'b0)
    );
`endif

endmodule


// =============================================================================
// branch_predictor
// =============================================================================
// A compact, synthesizable dynamic predictor for the single-issue in-order CPU:
//   * 64-entry direct-mapped BTB, indexed by PC[7:2], tagged by PC[31:8]
//   * 64-entry local 2-bit saturating-counter BHT for conditional branches
//   * 8-entry return-address stack (RAS) for ABI-standard returns
//
// Notes:
//   - Query uses virtual PC, matching the architectural PC carried IF->ID.
//   - Tables are updated only from a resolved ID-stage control transfer.
//   - RAS updates are non-speculative.  This CPU has only a one-entry IF
//     buffer, so this avoids recovery bookkeeping while still predicting normal
//     call/return chains correctly after the call is resolved.
//   - `DISABLE_BRANCH_PREDICTOR and the legacy `DISABLE_BTFNT both force the
//     original sequential PC+4 policy for A/B regression.
// =============================================================================
module branch_predictor(
    input         clk,
    input         reset,
    input  [31:0] query_pc,
    output        pred_taken,
    output [31:0] pred_nextpc,
    output        pred_taken_slot1,
    output [31:0] pred_nextpc_slot1,

    input         update_en,
    input  [31:0] update_pc,
    input         update_is_cond,
    input         update_taken,
    input  [31:0] update_target,
    input         update_is_call,
    input         update_is_return,
    input         update_is_indirect
);

    // Keep the original predictor capacity so IPC is not traded away merely to
    // close timing.  The training interface is pipelined by one clock below.
    localparam BTB_ENTRIES = 512;
    localparam BTB_IDX_W   = 9;
    localparam BTB_TAG_W   = 21;
    localparam BHT_ENTRIES = 512;
    localparam BHT_IDX_W   = 9;
    localparam GHR_W       = 9;
    localparam RAS_DEPTH   = 16;
    localparam RAS_PTR_W   = 4;

    localparam [1:0] BTB_COND     = 2'b00;
    localparam [1:0] BTB_DIRECT   = 2'b01;
    localparam [1:0] BTB_INDIRECT = 2'b10;
    localparam [1:0] BTB_RETURN   = 2'b11;

    localparam BTB_ENTRY_W = BTB_TAG_W + 32 + 2;
    (* ram_style = "distributed" *) reg [BTB_ENTRY_W-1:0] btb_mem [0:BTB_ENTRIES-1];
    reg [BTB_ENTRIES-1:0] btb_valid;

    (* ram_style = "distributed" *) reg [1:0] bht_mem [0:BHT_ENTRIES-1];
    reg [BHT_ENTRIES-1:0] bht_valid;
    reg [GHR_W-1:0] ghr;

    (* ram_style = "distributed" *) reg [31:0] ras_stack [0:RAS_DEPTH-1];
    reg [RAS_PTR_W-1:0] ras_sp;
    reg [RAS_PTR_W:0]   ras_count;

    wire [BTB_IDX_W-1:0] query_idx       = query_pc[10:2];
    wire [BTB_TAG_W-1:0] query_tag       = query_pc[31:11];
    // Use a PC-local direction table.  This avoids sending one branch through
    // many weak counters as global history changes and removes the XOR from
    // the fetch critical path.
    wire [BHT_IDX_W-1:0] query_bht_idx   = query_pc[10:2];
    wire [BTB_ENTRY_W-1:0] query_entry   = btb_mem[query_idx];
    wire [BTB_TAG_W-1:0] query_entry_tag = query_entry[BTB_ENTRY_W-1 -: BTB_TAG_W];
    wire [31:0] query_entry_target       = query_entry[33:2];
    wire [1:0]  query_entry_type         = query_entry[1:0];
    wire btb_hit = btb_valid[query_idx] && (query_entry_tag == query_tag);

    wire [1:0] query_bht_state = bht_valid[query_bht_idx] ?
                                 bht_mem[query_bht_idx] : 2'b01;

    wire ras_nonempty = (ras_count != 0);
    wire [RAS_PTR_W-1:0] ras_top_idx = ras_sp - {{(RAS_PTR_W-1){1'b0}}, 1'b1};
    wire [31:0] ras_top = ras_stack[ras_top_idx];

    wire [31:0] query_target =
        (query_entry_type == BTB_RETURN && ras_nonempty) ? ras_top : query_entry_target;

`ifdef DISABLE_BRANCH_PREDICTOR
    wire query_taken = 1'b0;
`elsif DISABLE_BTFNT
    wire query_taken = 1'b0;
`else
    wire query_taken = btb_hit &&
        ((query_entry_type == BTB_COND) ? query_bht_state[1] : 1'b1);
`endif

    assign pred_taken  = query_taken;
    assign pred_nextpc = query_taken ? query_target : (query_pc + 32'd4);

    // M18.1: an independently tagged lookup for the younger word of a dual
    // fetch.  The old frontend forced this word to not-taken, which converted
    // every trained second-slot branch into a needless redirect.  Both words
    // use the same committed GHR because prediction state is not speculative.
    wire [31:0] query_pc_slot1 = query_pc + 32'd4;
    wire [BTB_IDX_W-1:0] query_idx_slot1 = query_pc_slot1[10:2];
    wire [BTB_TAG_W-1:0] query_tag_slot1 = query_pc_slot1[31:11];
    wire [BHT_IDX_W-1:0] query_bht_idx_slot1 = query_pc_slot1[10:2];
    wire [BTB_ENTRY_W-1:0] query_entry_slot1 = btb_mem[query_idx_slot1];
    wire [BTB_TAG_W-1:0] query_entry_tag_slot1 =
        query_entry_slot1[BTB_ENTRY_W-1 -: BTB_TAG_W];
    wire [31:0] query_entry_target_slot1 = query_entry_slot1[33:2];
    wire [1:0] query_entry_type_slot1 = query_entry_slot1[1:0];
    wire btb_hit_slot1 = btb_valid[query_idx_slot1] &&
                         (query_entry_tag_slot1 == query_tag_slot1);
    wire [1:0] query_bht_state_slot1 = bht_valid[query_bht_idx_slot1] ?
                                       bht_mem[query_bht_idx_slot1] : 2'b01;
    wire [31:0] query_target_slot1 =
        (query_entry_type_slot1 == BTB_RETURN && ras_nonempty) ?
        ras_top : query_entry_target_slot1;

`ifdef DISABLE_BRANCH_PREDICTOR
    wire query_taken_slot1 = 1'b0;
`elsif DISABLE_BTFNT
    wire query_taken_slot1 = 1'b0;
`else
    wire query_taken_slot1 = btb_hit_slot1 &&
        ((query_entry_type_slot1 == BTB_COND) ?
         query_bht_state_slot1[1] : 1'b1);
`endif

    assign pred_taken_slot1 = query_taken_slot1;
    assign pred_nextpc_slot1 = query_taken_slot1 ? query_target_slot1 :
                                                   (query_pc + 32'd8);

    // -------------------------------------------------------------------------
    // One-stage training pipeline
    // -------------------------------------------------------------------------
    // Previously, update_en/update_pc/update_type came directly from the current
    // ID instruction.  That made the complete decode, hazard, branch comparison
    // and commit cone terminate at bht_valid/ghr/BTB write controls.  The routed
    // report showed ~8.8 ns of routing on this path.  Predictor training is not
    // architecturally visible, so delaying it by one cycle is safe.  Throughput
    // remains one update per cycle, including consecutive branches.
    reg        upd_valid_r;
    reg [31:0] upd_pc_r;
    reg        upd_is_cond_r;
    reg        upd_taken_r;
    reg [31:0] upd_target_r;
    reg        upd_is_call_r;
    reg        upd_is_return_r;
    reg        upd_is_indirect_r;

    wire [BTB_IDX_W-1:0] upd_idx     = upd_pc_r[10:2];
    wire [BTB_TAG_W-1:0] upd_tag     = upd_pc_r[31:11];
    wire [BHT_IDX_W-1:0] upd_bht_idx = upd_pc_r[10:2];
    wire [1:0] upd_type = upd_is_return_r   ? BTB_RETURN :
                          upd_is_cond_r     ? BTB_COND :
                          upd_is_indirect_r ? BTB_INDIRECT : BTB_DIRECT;

    wire [1:0] old_bht_state = bht_valid[upd_bht_idx] ?
                               bht_mem[upd_bht_idx] : 2'b01;
    wire [1:0] inc_bht_state = (old_bht_state == 2'b11) ?
                               2'b11 : old_bht_state + 2'b01;
    wire [1:0] dec_bht_state = (old_bht_state == 2'b00) ?
                               2'b00 : old_bht_state - 2'b01;

    always @(posedge clk) begin
        if (reset) begin
            btb_valid        <= {BTB_ENTRIES{1'b0}};
            bht_valid        <= {BHT_ENTRIES{1'b0}};
            ghr              <= {GHR_W{1'b0}};
            ras_sp           <= {RAS_PTR_W{1'b0}};
            ras_count        <= {(RAS_PTR_W+1){1'b0}};
            upd_valid_r      <= 1'b0;
            upd_pc_r         <= 32'b0;
            upd_is_cond_r    <= 1'b0;
            upd_taken_r      <= 1'b0;
            upd_target_r     <= 32'b0;
            upd_is_call_r    <= 1'b0;
            upd_is_return_r  <= 1'b0;
            upd_is_indirect_r<= 1'b0;
        end
        else begin
            // Capture the next training event.  Nonblocking assignment means the
            // table update below consumes the previous cycle's registered event.
            //
            // Round-7 timing fix: payload registers sample unconditionally and
            // upd_valid_r is their sole validity bit.  The old if(update_en) form
            // mapped the full ID/MMU/exception cone onto the CE pins of more than
            // 80 payload flops; the routed 10.833 ns report showed all top-10 CPU
            // setup paths ending at upd_pc_r[*]/CE.  Unconditional sampling removes
            // that high-fanout CE net without changing update throughput or state.
            upd_valid_r       <= update_en;
            upd_pc_r          <= update_pc;
            upd_is_cond_r     <= update_is_cond;
            upd_taken_r       <= update_taken;
            upd_target_r      <= update_target;
            upd_is_call_r     <= update_is_call;
            upd_is_return_r   <= update_is_return;
            upd_is_indirect_r <= update_is_indirect;

            if (upd_valid_r) begin
                btb_valid[upd_idx] <= 1'b1;
                btb_mem[upd_idx]   <= {upd_tag, upd_target_r, upd_type};

                if (upd_is_cond_r) begin
                    bht_valid[upd_bht_idx] <= 1'b1;
                    bht_mem[upd_bht_idx]   <= upd_taken_r ? inc_bht_state : dec_bht_state;
                    ghr <= {ghr[GHR_W-2:0], upd_taken_r};
                end

                if (upd_is_call_r) begin
                    ras_stack[ras_sp] <= upd_pc_r + 32'd4;
                    ras_sp <= ras_sp + {{(RAS_PTR_W-1){1'b0}}, 1'b1};
                    if (ras_count < RAS_DEPTH)
                        ras_count <= ras_count + {{RAS_PTR_W{1'b0}}, 1'b1};
                end
                else if (upd_is_return_r && ras_nonempty) begin
                    ras_sp    <= ras_sp - {{(RAS_PTR_W-1){1'b0}}, 1'b1};
                    ras_count <= ras_count - {{RAS_PTR_W{1'b0}}, 1'b1};
                end
            end
        end
    end
endmodule
