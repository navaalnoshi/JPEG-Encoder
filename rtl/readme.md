# <center> ***RTL Module Descriptions***

| Module Name       | Functionality Summary |
|------------------|------------------------|
|[**jpeg_top.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/top_level_module.sv)   | Top-level controller. Manages module sequencing, block enabling, input/output flow, and global reset/clock synchronization. |
|[ **rgb2ycbcr.v**](https://github.com/navaalnoshi/JPEG-Encoder/tree/main/rtl/1_rbg2ycrcb)   | Converts 24-bit RGB input into Y, Cb, and Cr color components using fixed-point arithmetic. Prepares image data for separate luminance and chrominance processing. |
|[ **pre_fifo.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/5_fifo/pre_fifo.sv)  | Buffers incoming data before processing. Helps align input pixel stream for color space conversion and DCT stages. |
|[ **sync_fifo_32.v** ](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/5_fifo/sync_fifo_32.sv)| 32-bit synchronous FIFO used to stage data across pipeline stages and ensure bit-aligned handoff between processing blocks. |
|[ **sync_fifo_ff.v** ](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/5_fifo/sync_fifo_ff.sv)| Simple flip-flop based FIFO. Used for low-latency handshaking between adjacent modules. |
|[ **fifo_out.v** ](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/5_fifo/fifo_out.sv)   | Buffers and synchronizes output data from Huffman modules before final bitstream packing. Handles valid signal control and output alignment. |
|[ **y_dct.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/2_dct/y_dct.sv)    | Applies 2D Discrete Cosine Transform to 8×8 Y (luminance) blocks. Converts spatial data into frequency domain using separable row-column 1D DCTs. |
|[ **cb_dct.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/2_dct/cr_dct.txt)      | Performs 2D DCT on 8×8 Cb (chrominance-blue) blocks. Structure and logic identical to `y_dct.v`. |
|[**cr_dct.v** ](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/2_dct/cb_dct.sv)     | Performs 2D DCT on 8×8 Cr (chrominance-red) blocks. Structure and logic identical to `y_dct.v`. |
|[ **y_quantizer.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/3_quantization/y_quantizer.sv) | Quantizes DCT-transformed Y coefficients using a 64-entry quantization matrix. Reduces precision based on compression level. |
|[ **cb_quantizer.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/3_quantization/cb_quantizer.sv)| Quantizes Cb coefficients using standard quantization tables or custom scaling factors. |
| [ **cr_quantizer.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/3_quantization/cr_quantizer.sv)| Quantizes Cr coefficients similarly. |
|[ **y_huff.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/4_huffman/y_huff.sv)     | Huffman encodes the quantized Y coefficients. Handles DC difference coding and AC run-length encoding using JPEG tables. |
|[**cb_huff.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/4_huffman/cb_huff.sv)  | Huffman encoder for Cb quantized blocks. Works identically to `y_huff.v` but with chroma tables. |
|[**cr_huff.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/4_huffman/cr_huff.sv)| Huffman encoder for Cr quantized blocks. |
|[ **yd_q_h.v** ](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/d_q_h/yd_q_h.sv)    | Combined Y DCT → Quantization → Huffman pipeline block. Reduces wiring and improves throughput for luminance path. |
|[ **cbd_q_h.v** ](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/d_q_h/cbd_q_h.sv)    | Combined Cb pipeline: DCT → Quantizer → Huffman. |
|[ **crd_q_h.v** ](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/d_q_h/crd_q_h.sv)    | Combined Cr pipeline: DCT → Quantizer → Huffman. |
|[**ff_checker.v**](https://github.com/navaalnoshi/JPEG-Encoder/blob/main/rtl/5_fifo/ff_checker.sv) | Ensures that `0xFF` byte stuffing rules are followed. Detects and corrects illegal JPEG byte sequences and appends end-of-file padding. |
