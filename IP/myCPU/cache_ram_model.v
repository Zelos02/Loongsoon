`ifdef VERILATOR

// ------------------------------------------------------------
// Behavioral RAM models for simulation only.
// Vivado keeps using its original Xilinx RAM IPs.
// Read behavior: synchronous, read-first on a write cycle.
// ------------------------------------------------------------

// 256 x 21-bit single-port tag/valid RAM
module tagv_ram (
    input  wire        clka,
    input  wire        ena,
    input  wire [0:0]  wea,
    input  wire [7:0]  addra,
    input  wire [20:0] dina,
    output reg  [20:0] douta
);

    reg [20:0] mem [0:255];

    always @(posedge clka) begin
        if (ena) begin
            // Read-first behavior on a simultaneous read/write.
            douta <= mem[addra];
            if (wea[0]) begin
                mem[addra] <= dina;
            end
        end
    end

endmodule


// 256 x 32-bit single-port data RAM with byte write enables
module data_bank_ram (
    input  wire        clka,
    input  wire        ena,
    input  wire [3:0]  wea,
    input  wire [7:0]  addra,
    input  wire [31:0] dina,
    output reg  [31:0] douta
);

    reg [31:0] mem [0:255];

    always @(posedge clka) begin
        if (ena) begin
            // Read-first behavior on a simultaneous read/write.
            douta <= mem[addra];

            if (wea[0]) mem[addra][7:0]   <= dina[7:0];
            if (wea[1]) mem[addra][15:8]  <= dina[15:8];
            if (wea[2]) mem[addra][23:16] <= dina[23:16];
            if (wea[3]) mem[addra][31:24] <= dina[31:24];
        end
    end

endmodule

`endif
