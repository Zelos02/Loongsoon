module regfile(
    input  wire        clk,
    // READ PORT 1
    input  wire [ 4:0] raddr1,
    output wire [31:0] rdata1,
    // READ PORT 2 (general rk/rd operand)
    input  wire [ 4:0] raddr2,
    output wire [31:0] rdata2,
    // Dedicated branch read ports.  Direct rj/rd addressing avoids placing the
    // generic src_reg_is_rd decode/mux in front of the branch compare RAM read.
    input  wire [ 4:0] br_raddr1,
    output wire [31:0] br_rdata1,
    input  wire [ 4:0] br_raddr2,
    output wire [31:0] br_rdata2,
    // SLOT-1 READ PORTS (dual-issue extension)
    input  wire [ 4:0] raddr3,
    output wire [31:0] rdata3,
    input  wire [ 4:0] raddr4,
    output wire [31:0] rdata4,
    // WRITE PORT 0 (older instruction)
    input  wire        we,
    input  wire [ 4:0] waddr,
    input  wire [31:0] wdata,
    // WRITE PORT 1 (younger instruction; wins on same-address WAW)
    input  wire        we2,
    input  wire [ 4:0] waddr2,
    input  wire [31:0] wdata2
);

// Four physical copies implement two general and two branch-local asynchronous read ports.
// Keeping one array named "rf" preserves the existing Difftest hierarchical references.
// Explicit distributed-RAM inference avoids a large 32:1 register mux tree and
// gives Vivado freedom to place each read copy near its consumer logic.
(* ram_style = "distributed" *) reg [31:0] rf        [31:0];
(* ram_style = "distributed" *) reg [31:0] rf_copy   [31:0];
(* ram_style = "distributed" *) reg [31:0] rf_br_rj  [31:0];
(* ram_style = "distributed" *) reg [31:0] rf_br_rd  [31:0];
(* ram_style = "distributed" *) reg [31:0] rf_slot1_rj[31:0];
(* ram_style = "distributed" *) reg [31:0] rf_slot1_rk[31:0];

always @(posedge clk) begin
    if (we && (waddr != 5'b0)) begin
        rf[waddr]          <= wdata;
        rf_copy[waddr]     <= wdata;
        rf_br_rj[waddr]    <= wdata;
        rf_br_rd[waddr]    <= wdata;
        rf_slot1_rj[waddr] <= wdata;
        rf_slot1_rk[waddr] <= wdata;
    end
    // Port 1 is the younger architectural write and therefore has priority
    // when both committing instructions target the same register.
    if (we2 && (waddr2 != 5'b0)) begin
        rf[waddr2]          <= wdata2;
        rf_copy[waddr2]     <= wdata2;
        rf_br_rj[waddr2]    <= wdata2;
        rf_br_rd[waddr2]    <= wdata2;
        rf_slot1_rj[waddr2] <= wdata2;
        rf_slot1_rk[waddr2] <= wdata2;
    end
end

wire [31:0] rdata1_mem    = rf[raddr1];
wire [31:0] rdata2_mem    = rf_copy[raddr2];
wire [31:0] br_rdata1_mem = rf_br_rj[br_raddr1];
wire [31:0] br_rdata2_mem = rf_br_rd[br_raddr2];
wire [31:0] rdata3_mem    = rf_slot1_rj[raddr3];
wire [31:0] rdata4_mem    = rf_slot1_rk[raddr4];

// Same-cycle WB bypass is local to each port and keeps architectural r0 zero.
assign rdata1 = (raddr1 == 5'b0) ? 32'b0 :
                (we2 && (waddr2 == raddr1) && (waddr2 != 5'b0)) ? wdata2 :
                (we && (waddr == raddr1) && (waddr != 5'b0)) ? wdata : rdata1_mem;

assign rdata2 = (raddr2 == 5'b0) ? 32'b0 :
                (we2 && (waddr2 == raddr2) && (waddr2 != 5'b0)) ? wdata2 :
                (we && (waddr == raddr2) && (waddr != 5'b0)) ? wdata : rdata2_mem;

// Keep WB bypass local to the branch copies too.  ID therefore needs only the
// MEM forwarding mux on its branch critical path; a second WB mux is redundant.
assign br_rdata1 = (br_raddr1 == 5'b0) ? 32'b0 :
                   (we2 && (waddr2 == br_raddr1) && (waddr2 != 5'b0)) ? wdata2 :
                   (we && (waddr == br_raddr1) && (waddr != 5'b0)) ? wdata : br_rdata1_mem;
assign br_rdata2 = (br_raddr2 == 5'b0) ? 32'b0 :
                   (we2 && (waddr2 == br_raddr2) && (waddr2 != 5'b0)) ? wdata2 :
                   (we  && (waddr  == br_raddr2) && (waddr  != 5'b0)) ? wdata  : br_rdata2_mem;

assign rdata3 = (raddr3 == 5'b0) ? 32'b0 :
                (we2 && (waddr2 == raddr3) && (waddr2 != 5'b0)) ? wdata2 :
                (we  && (waddr  == raddr3) && (waddr  != 5'b0)) ? wdata  : rdata3_mem;
assign rdata4 = (raddr4 == 5'b0) ? 32'b0 :
                (we2 && (waddr2 == raddr4) && (waddr2 != 5'b0)) ? wdata2 :
                (we  && (waddr  == raddr4) && (waddr  != 5'b0)) ? wdata  : rdata4_mem;

endmodule
