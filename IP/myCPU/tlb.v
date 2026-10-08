//==============================================================
// TLB module for LoongArch lab task 17
// - 2 search ports: s0 for fetch, s1 for load/store
// - 1 write port: TLBWR/TLBFILL
// - 1 read port: TLBRD
// - INVTLB invalidation support
//==============================================================
module tlb
#(
    parameter TLBNUM = 16
)
(
    input  wire                      clk,

    // search port 0 (for fetch)
    input  wire [              18:0] s0_vppn,
    input  wire                      s0_va_bit12,
    input  wire [               9:0] s0_asid,
    output wire                      s0_found,
    output wire [$clog2(TLBNUM)-1:0] s0_index,
    output wire [              19:0] s0_ppn,
    output wire [               5:0] s0_ps,
    output wire [               1:0] s0_plv,
    output wire [               1:0] s0_mat,
    output wire                      s0_d,
    output wire                      s0_v,

    // search port 1 (for load/store)
    input  wire [              18:0] s1_vppn,
    input  wire                      s1_va_bit12,
    input  wire [               9:0] s1_asid,
    output wire                      s1_found,
    output wire [$clog2(TLBNUM)-1:0] s1_index,
    output wire [              19:0] s1_ppn,
    output wire [               5:0] s1_ps,
    output wire [               1:0] s1_plv,
    output wire [               1:0] s1_mat,
    output wire                      s1_d,
    output wire                      s1_v,

    // invtlb opcode
    input  wire                      invtlb_valid,
    input  wire [               4:0] invtlb_op,

    // write port
    input  wire                      we,
    input  wire [$clog2(TLBNUM)-1:0] w_index,
    input  wire                      w_e,
    input  wire [              18:0] w_vppn,
    input  wire [               5:0] w_ps,
    input  wire [               9:0] w_asid,
    input  wire                      w_g,
    input  wire [              19:0] w_ppn0,
    input  wire [               1:0] w_plv0,
    input  wire [               1:0] w_mat0,
    input  wire                      w_d0,
    input  wire                      w_v0,
    input  wire [              19:0] w_ppn1,
    input  wire [               1:0] w_plv1,
    input  wire [               1:0] w_mat1,
    input  wire                      w_d1,
    input  wire                      w_v1,

    // read port
    input  wire [$clog2(TLBNUM)-1:0] r_index,
    output wire                      r_e,
    output wire [              18:0] r_vppn,
    output wire [               5:0] r_ps,
    output wire [               9:0] r_asid,
    output wire                      r_g,
    output wire [              19:0] r_ppn0,
    output wire [               1:0] r_plv0,
    output wire [               1:0] r_mat0,
    output wire                      r_d0,
    output wire                      r_v0,
    output wire [              19:0] r_ppn1,
    output wire [               1:0] r_plv1,
    output wire [               1:0] r_mat1,
    output wire                      r_d1,
    output wire                      r_v1
);

localparam INDEX_W = $clog2(TLBNUM);

//==============================================================
// TLB storage
// LoongArch lab only uses 4KB and 4MB pages here.
// Internally tlb_ps4MB = 1 means 4MB, 0 means 4KB.
//==============================================================
reg  [TLBNUM-1:0] tlb_e;
reg  [TLBNUM-1:0] tlb_ps4MB;

reg  [18:0] tlb_vppn [TLBNUM-1:0];
reg  [ 9:0] tlb_asid [TLBNUM-1:0];
reg         tlb_g    [TLBNUM-1:0];

reg  [19:0] tlb_ppn0 [TLBNUM-1:0];
reg  [ 1:0] tlb_plv0 [TLBNUM-1:0];
reg  [ 1:0] tlb_mat0 [TLBNUM-1:0];
reg         tlb_d0   [TLBNUM-1:0];
reg         tlb_v0   [TLBNUM-1:0];

reg  [19:0] tlb_ppn1 [TLBNUM-1:0];
reg  [ 1:0] tlb_plv1 [TLBNUM-1:0];
reg  [ 1:0] tlb_mat1 [TLBNUM-1:0];
reg         tlb_d1   [TLBNUM-1:0];
reg         tlb_v1   [TLBNUM-1:0];

integer init_i;
initial begin : init_tlb_entries
    for (init_i = 0; init_i < TLBNUM; init_i = init_i + 1) begin
        tlb_e    [init_i] = 1'b0;
        tlb_ps4MB[init_i] = 1'b0;
        tlb_vppn [init_i] = 19'b0;
        tlb_asid [init_i] = 10'b0;
        tlb_g    [init_i] = 1'b0;
        tlb_ppn0 [init_i] = 20'b0;
        tlb_plv0 [init_i] = 2'b0;
        tlb_mat0 [init_i] = 2'b0;
        tlb_d0   [init_i] = 1'b0;
        tlb_v0   [init_i] = 1'b0;
        tlb_ppn1 [init_i] = 20'b0;
        tlb_plv1 [init_i] = 2'b0;
        tlb_mat1 [init_i] = 2'b0;
        tlb_d1   [init_i] = 1'b0;
        tlb_v1   [init_i] = 1'b0;
    end
end

//==============================================================
// Match logic
// 4KB page : compare full VPPN[18:0]
// 4MB page : compare VPPN[18:10], ignore VPPN[9:0]
// ASID matches when ASID is equal or entry is global.
// E must be 1 to hit.
//==============================================================
wire [TLBNUM-1:0] match0;
wire [TLBNUM-1:0] match1;
wire [TLBNUM-1:0] inv_match;

genvar gi;
generate
    for (gi = 0; gi < TLBNUM; gi = gi + 1) begin : gen_match
        wire vppn_match0;
        wire vppn_match1;
        wire asid_match0;
        wire asid_match1;

        assign vppn_match0 = tlb_ps4MB[gi] ?
                              (s0_vppn[18:9] == tlb_vppn[gi][18:9]) :
                              (s0_vppn[18:0] == tlb_vppn[gi][18:0]);
        assign vppn_match1 = tlb_ps4MB[gi] ?
                              (s1_vppn[18:9] == tlb_vppn[gi][18:9]) :
                              (s1_vppn[18:0] == tlb_vppn[gi][18:0]);

        assign asid_match0 = (s0_asid == tlb_asid[gi]) || tlb_g[gi];
        assign asid_match1 = (s1_asid == tlb_asid[gi]) || tlb_g[gi];

        assign match0[gi] = tlb_e[gi] && vppn_match0 && asid_match0;
        assign match1[gi] = tlb_e[gi] && vppn_match1 && asid_match1;

        // INVTLB matching uses search port 1 inputs.
        // op 0/1: all entries
        // op 2  : global entries
        // op 3  : non-global entries
        // op 4  : non-global entries with matching ASID
        // op 5  : non-global entries with matching ASID and VPPN
        // op 6  : entries with matching VPPN and either global or matching ASID
        assign inv_match[gi] =
               (invtlb_op == 5'd0) ||
               (invtlb_op == 5'd1) ||
              ((invtlb_op == 5'd2) &&  tlb_g[gi]) ||
              ((invtlb_op == 5'd3) && !tlb_g[gi]) ||
              ((invtlb_op == 5'd4) && !tlb_g[gi] && (s1_asid == tlb_asid[gi])) ||
              ((invtlb_op == 5'd5) && !tlb_g[gi] && (s1_asid == tlb_asid[gi]) && vppn_match1) ||
              ((invtlb_op == 5'd6) && ((tlb_g[gi]) || (s1_asid == tlb_asid[gi])) && vppn_match1);
    end
endgenerate

//==============================================================
// Search result mux: choose the lowest-index matched entry.
// For 4KB pages, odd/even page is selected by VA bit 12.
// For 4MB pages, odd/even page is selected by VA bit 21,
// which is s_vppn[8] because s_vppn = VA[31:13].
//==============================================================
reg                      s0_found_r;
reg [INDEX_W-1:0]        s0_index_r;
reg [19:0]               s0_ppn_r;
reg [5 :0]               s0_ps_r;
reg [1 :0]               s0_plv_r;
reg [1 :0]               s0_mat_r;
reg                      s0_d_r;
reg                      s0_v_r;

reg                      s1_found_r;
reg [INDEX_W-1:0]        s1_index_r;
reg [19:0]               s1_ppn_r;
reg [5 :0]               s1_ps_r;
reg [1 :0]               s1_plv_r;
reg [1 :0]               s1_mat_r;
reg                      s1_d_r;
reg                      s1_v_r;

integer si;
always @(*) begin
    // exp19 opt13: assume software keeps TLB entries unique and use a parallel
    // one-hot OR mux instead of a lowest-index priority chain.  This shortens
    // the s0 search path used by fetch-side exception checks.
    s0_found_r = |match0;
    s0_index_r = {INDEX_W{1'b0}};
    s0_ppn_r   = 20'b0;
    s0_ps_r    = 6'b0;
    s0_plv_r   = 2'b0;
    s0_mat_r   = 2'b0;
    s0_d_r     = 1'b0;
    s0_v_r     = 1'b0;

    for (si = 0; si < TLBNUM; si = si + 1) begin
        if (match0[si]) begin
            s0_index_r = s0_index_r | si[INDEX_W-1:0];
            s0_ps_r    = s0_ps_r    | (tlb_ps4MB[si] ? 6'h15 : 6'h0c);

            if (tlb_ps4MB[si] ? s0_vppn[8] : s0_va_bit12) begin
                s0_ppn_r = s0_ppn_r | tlb_ppn1[si];
                s0_plv_r = s0_plv_r | tlb_plv1[si];
                s0_mat_r = s0_mat_r | tlb_mat1[si];
                s0_d_r   = s0_d_r   | tlb_d1  [si];
                s0_v_r   = s0_v_r   | tlb_v1  [si];
            end
            else begin
                s0_ppn_r = s0_ppn_r | tlb_ppn0[si];
                s0_plv_r = s0_plv_r | tlb_plv0[si];
                s0_mat_r = s0_mat_r | tlb_mat0[si];
                s0_d_r   = s0_d_r   | tlb_d0  [si];
                s0_v_r   = s0_v_r   | tlb_v0  [si];
            end
        end
    end
end

integer sj;
always @(*) begin
    // exp19 opt13: parallel one-hot OR mux for s1 as well.  It removes the
    // match -> found -> next-entry dependency chain from data-side MMU timing.
    s1_found_r = |match1;
    s1_index_r = {INDEX_W{1'b0}};
    s1_ppn_r   = 20'b0;
    s1_ps_r    = 6'b0;
    s1_plv_r   = 2'b0;
    s1_mat_r   = 2'b0;
    s1_d_r     = 1'b0;
    s1_v_r     = 1'b0;

    for (sj = 0; sj < TLBNUM; sj = sj + 1) begin
        if (match1[sj]) begin
            s1_index_r = s1_index_r | sj[INDEX_W-1:0];
            s1_ps_r    = s1_ps_r    | (tlb_ps4MB[sj] ? 6'h15 : 6'h0c);

            if (tlb_ps4MB[sj] ? s1_vppn[8] : s1_va_bit12) begin
                s1_ppn_r = s1_ppn_r | tlb_ppn1[sj];
                s1_plv_r = s1_plv_r | tlb_plv1[sj];
                s1_mat_r = s1_mat_r | tlb_mat1[sj];
                s1_d_r   = s1_d_r   | tlb_d1  [sj];
                s1_v_r   = s1_v_r   | tlb_v1  [sj];
            end
            else begin
                s1_ppn_r = s1_ppn_r | tlb_ppn0[sj];
                s1_plv_r = s1_plv_r | tlb_plv0[sj];
                s1_mat_r = s1_mat_r | tlb_mat0[sj];
                s1_d_r   = s1_d_r   | tlb_d0  [sj];
                s1_v_r   = s1_v_r   | tlb_v0  [sj];
            end
        end
    end
end

assign s0_found = s0_found_r;
assign s0_index = s0_index_r;
assign s0_ppn   = s0_ppn_r;
assign s0_ps    = s0_ps_r;
assign s0_plv   = s0_plv_r;
assign s0_mat   = s0_mat_r;
assign s0_d     = s0_d_r;
assign s0_v     = s0_v_r;

assign s1_found = s1_found_r;
assign s1_index = s1_index_r;
assign s1_ppn   = s1_ppn_r;
assign s1_ps    = s1_ps_r;
assign s1_plv   = s1_plv_r;
assign s1_mat   = s1_mat_r;
assign s1_d     = s1_d_r;
assign s1_v     = s1_v_r;

//==============================================================
// Read port
//==============================================================
assign r_e     = tlb_e    [r_index];
assign r_vppn  = tlb_vppn [r_index];
assign r_ps    = tlb_ps4MB[r_index] ? 6'h15 : 6'h0c;
assign r_asid  = tlb_asid [r_index];
assign r_g     = tlb_g    [r_index];

assign r_ppn0  = tlb_ppn0 [r_index];
assign r_plv0  = tlb_plv0 [r_index];
assign r_mat0  = tlb_mat0 [r_index];
assign r_d0    = tlb_d0   [r_index];
assign r_v0    = tlb_v0   [r_index];

assign r_ppn1  = tlb_ppn1 [r_index];
assign r_plv1  = tlb_plv1 [r_index];
assign r_mat1  = tlb_mat1 [r_index];
assign r_d1    = tlb_d1   [r_index];
assign r_v1    = tlb_v1   [r_index];

//==============================================================
// Write and invalidation port
//==============================================================
integer wi;
always @(posedge clk) begin
    if (we) begin
        tlb_e    [w_index] <= w_e;
        tlb_ps4MB[w_index] <= (w_ps == 6'h15);

        tlb_vppn [w_index] <= w_vppn;
        tlb_asid [w_index] <= w_asid;
        tlb_g    [w_index] <= w_g;

        tlb_ppn0 [w_index] <= w_ppn0;
        tlb_plv0 [w_index] <= w_plv0;
        tlb_mat0 [w_index] <= w_mat0;
        tlb_d0   [w_index] <= w_d0;
        tlb_v0   [w_index] <= w_v0;

        tlb_ppn1 [w_index] <= w_ppn1;
        tlb_plv1 [w_index] <= w_plv1;
        tlb_mat1 [w_index] <= w_mat1;
        tlb_d1   [w_index] <= w_d1;
        tlb_v1   [w_index] <= w_v1;
    end

    if (invtlb_valid) begin
        for (wi = 0; wi < TLBNUM; wi = wi + 1) begin
            if (inv_match[wi]) begin
                tlb_e[wi] <= 1'b0;
            end
        end
    end
end

endmodule
