// ============================================================================
// Module: audio_fifo
// Description: Dual-Clock FIFO for Audio Sample Cross-Domain Buffering
//
// This module expects the user to regenerate a DCFIFO via Quartus MegaWizard:
//
//   Menu: Tools → MegaWizard Plug-In Manager → FIFO
//   Settings:
//     - Type: DCFIFO (Dual Clock FIFO)
//     - Data width:  24 bits
//     - Depth:        256 words
//     - Device:       Cyclone IV E
//     - Output ports to ENABLE:
//         [✓] rdempty  — read-side empty flag
//         [✓] wrfull   — write-side full flag
//         [✓] rdusedw  — read-side used words (8 bits)
//         [✓] wrusedw  — write-side used words (8 bits)
//     - Save as: audio_fifo.v (in ip_core/audio_fifo/)
//
// Note: Depth reduced from 1024 to 256 to save M9K resources.
//       At 48kHz sample rate, 256 words = 5.3ms buffer, sufficient for
//       VGA display (60fps = 16.7ms frame period, continuous read).
//
// ============================================================================

// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on

module audio_fifo (
    input  wire [23:0]  data,           // Write data (24-bit audio sample)
    input  wire         rdclk,          // Read clock (VGA pixel clock, 40MHz)
    input  wire         rdreq,          // Read request
    input  wire         wrclk,          // Write clock (BCLK, 3.072MHz)
    input  wire         wrreq,          // Write request
    output wire [23:0]  q,              // Read data
    output wire         rdempty,        // FIFO empty (read side)
    output wire         wrfull,         // FIFO full (write side)
    output wire [7:0]   rdusedw,       // Read-side used words (8 bits for depth 256)
    output wire [7:0]   wrusedw        // Write-side used words (8 bits for depth 256)
);

    // ========================================================================
    // Altera DCFIFO Primitive
    // ========================================================================
    // This is a direct instantiation of the altera_mf dcfifo.
    // Parameters match the existing scfifo design for Cyclone IV E.

    wire [23:0] sub_wire0;
    wire sub_wire1;   // rdempty
    wire sub_wire2;   // wrfull
    wire [7:0] sub_wire3;  // rdusedw (8 bits for depth 256)
    wire [7:0] sub_wire4;  // wrusedw (8 bits for depth 256)

    assign q        = sub_wire0[23:0];
    assign rdempty  = sub_wire1;
    assign wrfull   = sub_wire2;
    assign rdusedw  = sub_wire3[7:0];
    assign wrusedw  = sub_wire4[7:0];

    dcfifo dcfifo_component (
        .aclr       (1'b0),
        .data       (data),
        .rdclk      (rdclk),
        .rdreq      (rdreq),
        .wrclk      (wrclk),
        .wrreq      (wrreq),
        .q          (sub_wire0),
        .rdempty    (sub_wire1),
        .rdfull     (),
        .rdusedw    (sub_wire3),
        .wrempty    (),
        .wrfull     (sub_wire2),
        .wrusedw    (sub_wire4)
    );
    defparam
        dcfifo_component.intended_device_family = "Cyclone IV E",
        dcfifo_component.lpm_numwords           = 256,
        dcfifo_component.lpm_showahead          = "OFF",
        dcfifo_component.lpm_type               = "dcfifo",
        dcfifo_component.lpm_width              = 24,
        dcfifo_component.lpm_widthu             = 8,
        dcfifo_component.overflow_checking      = "ON",
        dcfifo_component.rdsync_delaypipe       = 4,
        dcfifo_component.underflow_checking     = "ON",
        dcfifo_component.use_eab                = "ON",
        dcfifo_component.wrsync_delaypipe       = 4;

endmodule
