// Copyright 1986-2016 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2016.4 (win64) Build 1756540 Mon Jan 23 19:11:23 MST 2017
// Date        : Mon Dec 04 17:23:40 2017
// Host        : cristianf-ro running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode funcsim -rename_top design_1_axi_gpio_0_1 -prefix
//               design_1_axi_gpio_0_1_ design_1_axi_gpio_0_1_sim_netlist.v
// Design      : design_1_axi_gpio_0_1
// Purpose     : This verilog netlist is a functional simulation representation of the design and should not be modified
//               or synthesized. This netlist cannot be used for SDF annotated simulation.
// Device      : xc7z020clg400-1
// --------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_axi_gpio_0_1_GPIO_Core
   (GPIO_DBus_i,
    GPIO_xferAck_i,
    gpio_xferAck_Reg,
    GPIO2_DBus_i,
    gpio2_io_t,
    gpio2_io_o,
    ip2bus_rdack_i,
    ip2bus_wrack_i_D1_reg,
    gpio_io_o,
    gpio_io_t,
    \Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 ,
    Read_Reg_Rst,
    Read_Reg_In,
    s_axi_aclk,
    bus2ip_reset,
    bus2ip_rnw_i_reg,
    \bus2ip_addr_i_reg[2] ,
    bus2ip_rnw,
    bus2ip_cs,
    Q,
    \bus2ip_addr_i_reg[3] ,
    gpio_io_i,
    gpio2_io_i,
    E,
    D,
    bus2ip_rnw_i_reg_0);
  output [6:0]GPIO_DBus_i;
  output GPIO_xferAck_i;
  output gpio_xferAck_Reg;
  output [0:0]GPIO2_DBus_i;
  output [0:0]gpio2_io_t;
  output [0:0]gpio2_io_o;
  output ip2bus_rdack_i;
  output ip2bus_wrack_i_D1_reg;
  output [6:0]gpio_io_o;
  output [6:0]gpio_io_t;
  output [6:0]\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 ;
  input Read_Reg_Rst;
  input [0:6]Read_Reg_In;
  input s_axi_aclk;
  input bus2ip_reset;
  input bus2ip_rnw_i_reg;
  input \bus2ip_addr_i_reg[2] ;
  input bus2ip_rnw;
  input bus2ip_cs;
  input [0:0]Q;
  input \bus2ip_addr_i_reg[3] ;
  input [6:0]gpio_io_i;
  input [0:0]gpio2_io_i;
  input [0:0]E;
  input [6:0]D;
  input [0:0]bus2ip_rnw_i_reg_0;

  wire [6:0]D;
  wire \Dual.READ_REG2_GEN[0].GPIO2_DBus_i[31]_i_1_n_0 ;
  wire \Dual.READ_REG2_GEN[0].GPIO2_DBus_i[31]_i_2_n_0 ;
  wire [6:0]\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 ;
  wire [0:0]E;
  wire [0:0]GPIO2_DBus_i;
  wire [6:0]GPIO_DBus_i;
  wire GPIO_xferAck_i;
  wire [0:0]Q;
  wire [0:6]Read_Reg_In;
  wire Read_Reg_Rst;
  wire \bus2ip_addr_i_reg[2] ;
  wire \bus2ip_addr_i_reg[3] ;
  wire bus2ip_cs;
  wire bus2ip_reset;
  wire bus2ip_rnw;
  wire bus2ip_rnw_i_reg;
  wire [0:0]bus2ip_rnw_i_reg_0;
  wire gpio2_Data_In;
  wire [0:0]gpio2_io_i;
  wire gpio2_io_i_d2;
  wire [0:0]gpio2_io_o;
  wire [0:0]gpio2_io_t;
  wire [6:0]gpio_io_i;
  wire [0:6]gpio_io_i_d2;
  wire [6:0]gpio_io_o;
  wire [6:0]gpio_io_t;
  wire gpio_xferAck_Reg;
  wire iGPIO_xferAck;
  wire ip2bus_rdack_i;
  wire ip2bus_wrack_i_D1_reg;
  wire s_axi_aclk;

  design_1_axi_gpio_0_1_cdc_sync \Dual.INPUT_DOUBLE_REGS4 
       (.gpio_io_i(gpio_io_i),
        .s_axi_aclk(s_axi_aclk),
        .scndry_vect_out({gpio_io_i_d2[0],gpio_io_i_d2[1],gpio_io_i_d2[2],gpio_io_i_d2[3],gpio_io_i_d2[4],gpio_io_i_d2[5],gpio_io_i_d2[6]}));
  design_1_axi_gpio_0_1_cdc_sync__parameterized0 \Dual.INPUT_DOUBLE_REGS5 
       (.gpio2_io_i(gpio2_io_i),
        .s_axi_aclk(s_axi_aclk),
        .scndry_vect_out(gpio2_io_i_d2));
  LUT6 #(
    .INIT(64'h000000E200000000)) 
    \Dual.READ_REG2_GEN[0].GPIO2_DBus_i[31]_i_1 
       (.I0(gpio2_Data_In),
        .I1(Q),
        .I2(gpio2_io_t),
        .I3(\bus2ip_addr_i_reg[3] ),
        .I4(\Dual.READ_REG2_GEN[0].GPIO2_DBus_i[31]_i_2_n_0 ),
        .I5(bus2ip_rnw),
        .O(\Dual.READ_REG2_GEN[0].GPIO2_DBus_i[31]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT2 #(
    .INIT(4'hE)) 
    \Dual.READ_REG2_GEN[0].GPIO2_DBus_i[31]_i_2 
       (.I0(gpio_xferAck_Reg),
        .I1(GPIO_xferAck_i),
        .O(\Dual.READ_REG2_GEN[0].GPIO2_DBus_i[31]_i_2_n_0 ));
  FDRE \Dual.READ_REG2_GEN[0].GPIO2_DBus_i_reg[31] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(\Dual.READ_REG2_GEN[0].GPIO2_DBus_i[31]_i_1_n_0 ),
        .Q(GPIO2_DBus_i),
        .R(1'b0));
  FDRE \Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(Read_Reg_In[0]),
        .Q(GPIO_DBus_i[6]),
        .R(Read_Reg_Rst));
  FDRE \Dual.READ_REG_GEN[1].GPIO_DBus_i_reg[26] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(Read_Reg_In[1]),
        .Q(GPIO_DBus_i[5]),
        .R(Read_Reg_Rst));
  FDRE \Dual.READ_REG_GEN[2].GPIO_DBus_i_reg[27] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(Read_Reg_In[2]),
        .Q(GPIO_DBus_i[4]),
        .R(Read_Reg_Rst));
  FDRE \Dual.READ_REG_GEN[3].GPIO_DBus_i_reg[28] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(Read_Reg_In[3]),
        .Q(GPIO_DBus_i[3]),
        .R(Read_Reg_Rst));
  FDRE \Dual.READ_REG_GEN[4].GPIO_DBus_i_reg[29] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(Read_Reg_In[4]),
        .Q(GPIO_DBus_i[2]),
        .R(Read_Reg_Rst));
  FDRE \Dual.READ_REG_GEN[5].GPIO_DBus_i_reg[30] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(Read_Reg_In[5]),
        .Q(GPIO_DBus_i[1]),
        .R(Read_Reg_Rst));
  FDRE \Dual.READ_REG_GEN[6].GPIO_DBus_i_reg[31] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(Read_Reg_In[6]),
        .Q(GPIO_DBus_i[0]),
        .R(Read_Reg_Rst));
  FDRE \Dual.gpio2_Data_In_reg[0] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio2_io_i_d2),
        .Q(gpio2_Data_In),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \Dual.gpio2_Data_Out_reg[0] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(\bus2ip_addr_i_reg[2] ),
        .Q(gpio2_io_o),
        .R(bus2ip_reset));
  FDSE #(
    .INIT(1'b1)) 
    \Dual.gpio2_OE_reg[0] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(bus2ip_rnw_i_reg),
        .Q(gpio2_io_t),
        .S(bus2ip_reset));
  FDRE \Dual.gpio_Data_In_reg[0] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i_d2[0]),
        .Q(\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 [6]),
        .R(1'b0));
  FDRE \Dual.gpio_Data_In_reg[1] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i_d2[1]),
        .Q(\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 [5]),
        .R(1'b0));
  FDRE \Dual.gpio_Data_In_reg[2] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i_d2[2]),
        .Q(\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 [4]),
        .R(1'b0));
  FDRE \Dual.gpio_Data_In_reg[3] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i_d2[3]),
        .Q(\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 [3]),
        .R(1'b0));
  FDRE \Dual.gpio_Data_In_reg[4] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i_d2[4]),
        .Q(\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 [2]),
        .R(1'b0));
  FDRE \Dual.gpio_Data_In_reg[5] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i_d2[5]),
        .Q(\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 [1]),
        .R(1'b0));
  FDRE \Dual.gpio_Data_In_reg[6] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i_d2[6]),
        .Q(\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 [0]),
        .R(1'b0));
  FDRE #(
    .INIT(1'b0)) 
    \Dual.gpio_Data_Out_reg[0] 
       (.C(s_axi_aclk),
        .CE(E),
        .D(D[6]),
        .Q(gpio_io_o[6]),
        .R(bus2ip_reset));
  FDRE #(
    .INIT(1'b0)) 
    \Dual.gpio_Data_Out_reg[1] 
       (.C(s_axi_aclk),
        .CE(E),
        .D(D[5]),
        .Q(gpio_io_o[5]),
        .R(bus2ip_reset));
  FDRE #(
    .INIT(1'b0)) 
    \Dual.gpio_Data_Out_reg[2] 
       (.C(s_axi_aclk),
        .CE(E),
        .D(D[4]),
        .Q(gpio_io_o[4]),
        .R(bus2ip_reset));
  FDRE #(
    .INIT(1'b0)) 
    \Dual.gpio_Data_Out_reg[3] 
       (.C(s_axi_aclk),
        .CE(E),
        .D(D[3]),
        .Q(gpio_io_o[3]),
        .R(bus2ip_reset));
  FDRE #(
    .INIT(1'b0)) 
    \Dual.gpio_Data_Out_reg[4] 
       (.C(s_axi_aclk),
        .CE(E),
        .D(D[2]),
        .Q(gpio_io_o[2]),
        .R(bus2ip_reset));
  FDRE #(
    .INIT(1'b0)) 
    \Dual.gpio_Data_Out_reg[5] 
       (.C(s_axi_aclk),
        .CE(E),
        .D(D[1]),
        .Q(gpio_io_o[1]),
        .R(bus2ip_reset));
  FDRE #(
    .INIT(1'b0)) 
    \Dual.gpio_Data_Out_reg[6] 
       (.C(s_axi_aclk),
        .CE(E),
        .D(D[0]),
        .Q(gpio_io_o[0]),
        .R(bus2ip_reset));
  FDSE #(
    .INIT(1'b1)) 
    \Dual.gpio_OE_reg[0] 
       (.C(s_axi_aclk),
        .CE(bus2ip_rnw_i_reg_0),
        .D(D[6]),
        .Q(gpio_io_t[6]),
        .S(bus2ip_reset));
  FDSE #(
    .INIT(1'b1)) 
    \Dual.gpio_OE_reg[1] 
       (.C(s_axi_aclk),
        .CE(bus2ip_rnw_i_reg_0),
        .D(D[5]),
        .Q(gpio_io_t[5]),
        .S(bus2ip_reset));
  FDSE #(
    .INIT(1'b1)) 
    \Dual.gpio_OE_reg[2] 
       (.C(s_axi_aclk),
        .CE(bus2ip_rnw_i_reg_0),
        .D(D[4]),
        .Q(gpio_io_t[4]),
        .S(bus2ip_reset));
  FDSE #(
    .INIT(1'b1)) 
    \Dual.gpio_OE_reg[3] 
       (.C(s_axi_aclk),
        .CE(bus2ip_rnw_i_reg_0),
        .D(D[3]),
        .Q(gpio_io_t[3]),
        .S(bus2ip_reset));
  FDSE #(
    .INIT(1'b1)) 
    \Dual.gpio_OE_reg[4] 
       (.C(s_axi_aclk),
        .CE(bus2ip_rnw_i_reg_0),
        .D(D[2]),
        .Q(gpio_io_t[2]),
        .S(bus2ip_reset));
  FDSE #(
    .INIT(1'b1)) 
    \Dual.gpio_OE_reg[5] 
       (.C(s_axi_aclk),
        .CE(bus2ip_rnw_i_reg_0),
        .D(D[1]),
        .Q(gpio_io_t[1]),
        .S(bus2ip_reset));
  FDSE #(
    .INIT(1'b1)) 
    \Dual.gpio_OE_reg[6] 
       (.C(s_axi_aclk),
        .CE(bus2ip_rnw_i_reg_0),
        .D(D[0]),
        .Q(gpio_io_t[0]),
        .S(bus2ip_reset));
  FDRE gpio_xferAck_Reg_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(GPIO_xferAck_i),
        .Q(gpio_xferAck_Reg),
        .R(bus2ip_reset));
  (* SOFT_HLUTNM = "soft_lutpair7" *) 
  LUT3 #(
    .INIT(8'h02)) 
    iGPIO_xferAck_i_1
       (.I0(bus2ip_cs),
        .I1(GPIO_xferAck_i),
        .I2(gpio_xferAck_Reg),
        .O(iGPIO_xferAck));
  FDRE iGPIO_xferAck_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(iGPIO_xferAck),
        .Q(GPIO_xferAck_i),
        .R(bus2ip_reset));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT2 #(
    .INIT(4'h8)) 
    ip2bus_rdack_i_D1_i_1
       (.I0(GPIO_xferAck_i),
        .I1(bus2ip_rnw),
        .O(ip2bus_rdack_i));
  (* SOFT_HLUTNM = "soft_lutpair8" *) 
  LUT2 #(
    .INIT(4'h2)) 
    ip2bus_wrack_i_D1_i_1
       (.I0(GPIO_xferAck_i),
        .I1(bus2ip_rnw),
        .O(ip2bus_wrack_i_D1_reg));
endmodule

module design_1_axi_gpio_0_1_address_decoder
   (\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ,
    s_axi_arready,
    s_axi_wready,
    \Dual.gpio2_OE_reg[0] ,
    D,
    \ip2bus_data_i_D1_reg[25] ,
    Read_Reg_In,
    E,
    \Dual.gpio_Data_Out_reg[0] ,
    Read_Reg_Rst,
    \Dual.gpio2_OE_reg[0]_0 ,
    \Dual.gpio2_Data_Out_reg[0] ,
    s_axi_aclk,
    Q,
    is_read,
    ip2bus_rdack_i_D1,
    is_write_reg,
    ip2bus_wrack_i_D1,
    \bus2ip_addr_i_reg[8] ,
    s_axi_wdata,
    GPIO2_DBus_i,
    bus2ip_rnw_i_reg,
    GPIO_DBus_i,
    gpio_io_t,
    \Dual.gpio_Data_In_reg[0] ,
    rst_reg,
    GPIO_xferAck_i,
    gpio_xferAck_Reg,
    start2,
    s_axi_aresetn,
    gpio2_io_t,
    gpio2_io_o);
  output \MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ;
  output s_axi_arready;
  output s_axi_wready;
  output \Dual.gpio2_OE_reg[0] ;
  output [6:0]D;
  output [6:0]\ip2bus_data_i_D1_reg[25] ;
  output [0:6]Read_Reg_In;
  output [0:0]E;
  output [0:0]\Dual.gpio_Data_Out_reg[0] ;
  output Read_Reg_Rst;
  output \Dual.gpio2_OE_reg[0]_0 ;
  output \Dual.gpio2_Data_Out_reg[0] ;
  input s_axi_aclk;
  input [3:0]Q;
  input is_read;
  input ip2bus_rdack_i_D1;
  input is_write_reg;
  input ip2bus_wrack_i_D1;
  input [2:0]\bus2ip_addr_i_reg[8] ;
  input [6:0]s_axi_wdata;
  input [0:0]GPIO2_DBus_i;
  input bus2ip_rnw_i_reg;
  input [6:0]GPIO_DBus_i;
  input [6:0]gpio_io_t;
  input [6:0]\Dual.gpio_Data_In_reg[0] ;
  input rst_reg;
  input GPIO_xferAck_i;
  input gpio_xferAck_Reg;
  input start2;
  input s_axi_aresetn;
  input [0:0]gpio2_io_t;
  input [0:0]gpio2_io_o;

  wire [6:0]D;
  wire \Dual.gpio2_Data_Out_reg[0] ;
  wire \Dual.gpio2_OE_reg[0] ;
  wire \Dual.gpio2_OE_reg[0]_0 ;
  wire [6:0]\Dual.gpio_Data_In_reg[0] ;
  wire [0:0]\Dual.gpio_Data_Out_reg[0] ;
  wire [0:0]E;
  wire [0:0]GPIO2_DBus_i;
  wire [6:0]GPIO_DBus_i;
  wire GPIO_xferAck_i;
  wire \MEM_DECODE_GEN[0].cs_out_i[0]_i_1_n_0 ;
  wire \MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ;
  wire [3:0]Q;
  wire [0:6]Read_Reg_In;
  wire Read_Reg_Rst;
  wire [2:0]\bus2ip_addr_i_reg[8] ;
  wire bus2ip_rnw_i_reg;
  wire [0:0]gpio2_io_o;
  wire [0:0]gpio2_io_t;
  wire [6:0]gpio_io_t;
  wire gpio_xferAck_Reg;
  wire [6:0]\ip2bus_data_i_D1_reg[25] ;
  wire ip2bus_rdack_i_D1;
  wire ip2bus_wrack_i_D1;
  wire is_read;
  wire is_write_reg;
  wire rst_reg;
  wire s_axi_aclk;
  wire s_axi_aresetn;
  wire s_axi_arready;
  wire [6:0]s_axi_wdata;
  wire s_axi_wready;
  wire start2;

  LUT6 #(
    .INIT(64'h000A0000000C0000)) 
    \Dual.READ_REG_GEN[0].GPIO_DBus_i[25]_i_1 
       (.I0(gpio_io_t[6]),
        .I1(\Dual.gpio_Data_In_reg[0] [6]),
        .I2(\bus2ip_addr_i_reg[8] [2]),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I5(\bus2ip_addr_i_reg[8] [0]),
        .O(Read_Reg_In[0]));
  LUT6 #(
    .INIT(64'h000A0000000C0000)) 
    \Dual.READ_REG_GEN[1].GPIO_DBus_i[26]_i_1 
       (.I0(gpio_io_t[5]),
        .I1(\Dual.gpio_Data_In_reg[0] [5]),
        .I2(\bus2ip_addr_i_reg[8] [2]),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I5(\bus2ip_addr_i_reg[8] [0]),
        .O(Read_Reg_In[1]));
  LUT6 #(
    .INIT(64'h000A0000000C0000)) 
    \Dual.READ_REG_GEN[2].GPIO_DBus_i[27]_i_1 
       (.I0(gpio_io_t[4]),
        .I1(\Dual.gpio_Data_In_reg[0] [4]),
        .I2(\bus2ip_addr_i_reg[8] [2]),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I5(\bus2ip_addr_i_reg[8] [0]),
        .O(Read_Reg_In[2]));
  LUT6 #(
    .INIT(64'h000A0000000C0000)) 
    \Dual.READ_REG_GEN[3].GPIO_DBus_i[28]_i_1 
       (.I0(gpio_io_t[3]),
        .I1(\Dual.gpio_Data_In_reg[0] [3]),
        .I2(\bus2ip_addr_i_reg[8] [2]),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I5(\bus2ip_addr_i_reg[8] [0]),
        .O(Read_Reg_In[3]));
  LUT6 #(
    .INIT(64'h000A0000000C0000)) 
    \Dual.READ_REG_GEN[4].GPIO_DBus_i[29]_i_1 
       (.I0(gpio_io_t[2]),
        .I1(\Dual.gpio_Data_In_reg[0] [2]),
        .I2(\bus2ip_addr_i_reg[8] [2]),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I5(\bus2ip_addr_i_reg[8] [0]),
        .O(Read_Reg_In[4]));
  LUT6 #(
    .INIT(64'h000A0000000C0000)) 
    \Dual.READ_REG_GEN[5].GPIO_DBus_i[30]_i_1 
       (.I0(gpio_io_t[1]),
        .I1(\Dual.gpio_Data_In_reg[0] [1]),
        .I2(\bus2ip_addr_i_reg[8] [2]),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I5(\bus2ip_addr_i_reg[8] [0]),
        .O(Read_Reg_In[5]));
  LUT4 #(
    .INIT(16'hFFF7)) 
    \Dual.READ_REG_GEN[6].GPIO_DBus_i[31]_i_1 
       (.I0(bus2ip_rnw_i_reg),
        .I1(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I2(GPIO_xferAck_i),
        .I3(gpio_xferAck_Reg),
        .O(Read_Reg_Rst));
  LUT6 #(
    .INIT(64'h000A0000000C0000)) 
    \Dual.READ_REG_GEN[6].GPIO_DBus_i[31]_i_2 
       (.I0(gpio_io_t[0]),
        .I1(\Dual.gpio_Data_In_reg[0] [0]),
        .I2(\bus2ip_addr_i_reg[8] [2]),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I5(\bus2ip_addr_i_reg[8] [0]),
        .O(Read_Reg_In[6]));
  LUT5 #(
    .INIT(32'hFFFE0002)) 
    \Dual.gpio2_Data_Out[0]_i_1 
       (.I0(D[6]),
        .I1(\bus2ip_addr_i_reg[8] [0]),
        .I2(\Dual.gpio2_OE_reg[0] ),
        .I3(bus2ip_rnw_i_reg),
        .I4(gpio2_io_o),
        .O(\Dual.gpio2_Data_Out_reg[0] ));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT3 #(
    .INIT(8'hDF)) 
    \Dual.gpio2_Data_Out[0]_i_2 
       (.I0(\bus2ip_addr_i_reg[8] [1]),
        .I1(\bus2ip_addr_i_reg[8] [2]),
        .I2(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .O(\Dual.gpio2_OE_reg[0] ));
  LUT5 #(
    .INIT(32'hFFEF0020)) 
    \Dual.gpio2_OE[0]_i_1 
       (.I0(D[6]),
        .I1(bus2ip_rnw_i_reg),
        .I2(\bus2ip_addr_i_reg[8] [0]),
        .I3(\Dual.gpio2_OE_reg[0] ),
        .I4(gpio2_io_t),
        .O(\Dual.gpio2_OE_reg[0]_0 ));
  LUT6 #(
    .INIT(64'hFFFFFFFF00000100)) 
    \Dual.gpio_Data_Out[0]_i_1 
       (.I0(bus2ip_rnw_i_reg),
        .I1(\bus2ip_addr_i_reg[8] [2]),
        .I2(\bus2ip_addr_i_reg[8] [1]),
        .I3(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I4(\bus2ip_addr_i_reg[8] [0]),
        .I5(rst_reg),
        .O(\Dual.gpio_Data_Out_reg[0] ));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT4 #(
    .INIT(16'hBA8A)) 
    \Dual.gpio_Data_Out[0]_i_2 
       (.I0(s_axi_wdata[0]),
        .I1(\bus2ip_addr_i_reg[8] [1]),
        .I2(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I3(s_axi_wdata[6]),
        .O(D[6]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'h08)) 
    \Dual.gpio_Data_Out[1]_i_1 
       (.I0(s_axi_wdata[5]),
        .I1(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I2(\bus2ip_addr_i_reg[8] [1]),
        .O(D[5]));
  LUT3 #(
    .INIT(8'h08)) 
    \Dual.gpio_Data_Out[2]_i_1 
       (.I0(s_axi_wdata[4]),
        .I1(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I2(\bus2ip_addr_i_reg[8] [1]),
        .O(D[4]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'h08)) 
    \Dual.gpio_Data_Out[3]_i_1 
       (.I0(s_axi_wdata[3]),
        .I1(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I2(\bus2ip_addr_i_reg[8] [1]),
        .O(D[3]));
  (* SOFT_HLUTNM = "soft_lutpair3" *) 
  LUT3 #(
    .INIT(8'h08)) 
    \Dual.gpio_Data_Out[4]_i_1 
       (.I0(s_axi_wdata[2]),
        .I1(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I2(\bus2ip_addr_i_reg[8] [1]),
        .O(D[2]));
  (* SOFT_HLUTNM = "soft_lutpair2" *) 
  LUT3 #(
    .INIT(8'h08)) 
    \Dual.gpio_Data_Out[5]_i_1 
       (.I0(s_axi_wdata[1]),
        .I1(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I2(\bus2ip_addr_i_reg[8] [1]),
        .O(D[1]));
  (* SOFT_HLUTNM = "soft_lutpair1" *) 
  LUT3 #(
    .INIT(8'h08)) 
    \Dual.gpio_Data_Out[6]_i_1 
       (.I0(s_axi_wdata[0]),
        .I1(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I2(\bus2ip_addr_i_reg[8] [1]),
        .O(D[0]));
  LUT6 #(
    .INIT(64'hFFFFFFFF00040000)) 
    \Dual.gpio_OE[0]_i_1 
       (.I0(bus2ip_rnw_i_reg),
        .I1(\bus2ip_addr_i_reg[8] [0]),
        .I2(\bus2ip_addr_i_reg[8] [2]),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I5(rst_reg),
        .O(E));
  LUT5 #(
    .INIT(32'h000E0000)) 
    \MEM_DECODE_GEN[0].cs_out_i[0]_i_1 
       (.I0(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I1(start2),
        .I2(s_axi_wready),
        .I3(s_axi_arready),
        .I4(s_axi_aresetn),
        .O(\MEM_DECODE_GEN[0].cs_out_i[0]_i_1_n_0 ));
  FDRE \MEM_DECODE_GEN[0].cs_out_i_reg[0] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(\MEM_DECODE_GEN[0].cs_out_i[0]_i_1_n_0 ),
        .Q(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .R(1'b0));
  LUT5 #(
    .INIT(32'h00000080)) 
    \ip2bus_data_i_D1[25]_i_1 
       (.I0(GPIO_DBus_i[6]),
        .I1(bus2ip_rnw_i_reg),
        .I2(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\bus2ip_addr_i_reg[8] [2]),
        .O(\ip2bus_data_i_D1_reg[25] [6]));
  LUT5 #(
    .INIT(32'h00000080)) 
    \ip2bus_data_i_D1[26]_i_1 
       (.I0(GPIO_DBus_i[5]),
        .I1(bus2ip_rnw_i_reg),
        .I2(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\bus2ip_addr_i_reg[8] [2]),
        .O(\ip2bus_data_i_D1_reg[25] [5]));
  LUT5 #(
    .INIT(32'h00000080)) 
    \ip2bus_data_i_D1[27]_i_1 
       (.I0(GPIO_DBus_i[4]),
        .I1(bus2ip_rnw_i_reg),
        .I2(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\bus2ip_addr_i_reg[8] [2]),
        .O(\ip2bus_data_i_D1_reg[25] [4]));
  LUT5 #(
    .INIT(32'h00000080)) 
    \ip2bus_data_i_D1[28]_i_1 
       (.I0(GPIO_DBus_i[3]),
        .I1(bus2ip_rnw_i_reg),
        .I2(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\bus2ip_addr_i_reg[8] [2]),
        .O(\ip2bus_data_i_D1_reg[25] [3]));
  LUT5 #(
    .INIT(32'h00000080)) 
    \ip2bus_data_i_D1[29]_i_1 
       (.I0(GPIO_DBus_i[2]),
        .I1(bus2ip_rnw_i_reg),
        .I2(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\bus2ip_addr_i_reg[8] [2]),
        .O(\ip2bus_data_i_D1_reg[25] [2]));
  (* SOFT_HLUTNM = "soft_lutpair0" *) 
  LUT5 #(
    .INIT(32'h00000080)) 
    \ip2bus_data_i_D1[30]_i_1 
       (.I0(GPIO_DBus_i[1]),
        .I1(bus2ip_rnw_i_reg),
        .I2(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I3(\bus2ip_addr_i_reg[8] [1]),
        .I4(\bus2ip_addr_i_reg[8] [2]),
        .O(\ip2bus_data_i_D1_reg[25] [1]));
  LUT6 #(
    .INIT(64'hABAAAAAAA8AAAAAA)) 
    \ip2bus_data_i_D1[31]_i_1 
       (.I0(GPIO2_DBus_i),
        .I1(\bus2ip_addr_i_reg[8] [2]),
        .I2(\bus2ip_addr_i_reg[8] [1]),
        .I3(\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 ),
        .I4(bus2ip_rnw_i_reg),
        .I5(GPIO_DBus_i[0]),
        .O(\ip2bus_data_i_D1_reg[25] [0]));
  LUT6 #(
    .INIT(64'hFFFFFFFF00020000)) 
    s_axi_arready_INST_0
       (.I0(Q[3]),
        .I1(Q[2]),
        .I2(Q[1]),
        .I3(Q[0]),
        .I4(is_read),
        .I5(ip2bus_rdack_i_D1),
        .O(s_axi_arready));
  LUT6 #(
    .INIT(64'hFFFFFFFF00020000)) 
    s_axi_wready_INST_0
       (.I0(Q[3]),
        .I1(Q[2]),
        .I2(Q[1]),
        .I3(Q[0]),
        .I4(is_write_reg),
        .I5(ip2bus_wrack_i_D1),
        .O(s_axi_wready));
endmodule

(* C_ALL_INPUTS = "0" *) (* C_ALL_INPUTS_2 = "1" *) (* C_ALL_OUTPUTS = "1" *) 
(* C_ALL_OUTPUTS_2 = "0" *) (* C_DOUT_DEFAULT = "0" *) (* C_DOUT_DEFAULT_2 = "0" *) 
(* C_FAMILY = "zynq" *) (* C_GPIO2_WIDTH = "1" *) (* C_GPIO_WIDTH = "7" *) 
(* C_INTERRUPT_PRESENT = "0" *) (* C_IS_DUAL = "1" *) (* C_S_AXI_ADDR_WIDTH = "9" *) 
(* C_S_AXI_DATA_WIDTH = "32" *) (* C_TRI_DEFAULT = "-1" *) (* C_TRI_DEFAULT_2 = "-1" *) 
(* downgradeipidentifiedwarnings = "yes" *) (* ip_group = "LOGICORE" *) 
module design_1_axi_gpio_0_1_axi_gpio
   (s_axi_aclk,
    s_axi_aresetn,
    s_axi_awaddr,
    s_axi_awvalid,
    s_axi_awready,
    s_axi_wdata,
    s_axi_wstrb,
    s_axi_wvalid,
    s_axi_wready,
    s_axi_bresp,
    s_axi_bvalid,
    s_axi_bready,
    s_axi_araddr,
    s_axi_arvalid,
    s_axi_arready,
    s_axi_rdata,
    s_axi_rresp,
    s_axi_rvalid,
    s_axi_rready,
    ip2intc_irpt,
    gpio_io_i,
    gpio_io_o,
    gpio_io_t,
    gpio2_io_i,
    gpio2_io_o,
    gpio2_io_t);
  (* max_fanout = "10000" *) (* sigis = "Clk" *) input s_axi_aclk;
  (* max_fanout = "10000" *) (* sigis = "Rst" *) input s_axi_aresetn;
  input [8:0]s_axi_awaddr;
  input s_axi_awvalid;
  output s_axi_awready;
  input [31:0]s_axi_wdata;
  input [3:0]s_axi_wstrb;
  input s_axi_wvalid;
  output s_axi_wready;
  output [1:0]s_axi_bresp;
  output s_axi_bvalid;
  input s_axi_bready;
  input [8:0]s_axi_araddr;
  input s_axi_arvalid;
  output s_axi_arready;
  output [31:0]s_axi_rdata;
  output [1:0]s_axi_rresp;
  output s_axi_rvalid;
  input s_axi_rready;
  (* sigis = "INTR_LEVEL_HIGH" *) output ip2intc_irpt;
  input [6:0]gpio_io_i;
  output [6:0]gpio_io_o;
  output [6:0]gpio_io_t;
  input [0:0]gpio2_io_i;
  output [0:0]gpio2_io_o;
  output [0:0]gpio2_io_t;

  wire \<const0> ;
  wire AXI_LITE_IPIF_I_n_10;
  wire AXI_LITE_IPIF_I_n_11;
  wire AXI_LITE_IPIF_I_n_12;
  wire AXI_LITE_IPIF_I_n_13;
  wire AXI_LITE_IPIF_I_n_14;
  wire AXI_LITE_IPIF_I_n_15;
  wire AXI_LITE_IPIF_I_n_30;
  wire AXI_LITE_IPIF_I_n_31;
  wire AXI_LITE_IPIF_I_n_33;
  wire AXI_LITE_IPIF_I_n_34;
  wire AXI_LITE_IPIF_I_n_7;
  wire [31:31]GPIO2_DBus_i;
  wire [6:0]GPIO_DBus;
  wire [25:31]GPIO_DBus_i;
  wire GPIO_xferAck_i;
  wire [0:6]Read_Reg_In;
  wire Read_Reg_Rst;
  wire [6:6]bus2ip_addr;
  wire bus2ip_cs;
  wire bus2ip_reset;
  wire bus2ip_rnw;
  wire [0:0]gpio2_io_i;
  wire [0:0]gpio2_io_o;
  wire [0:0]gpio2_io_t;
  wire [0:6]gpio_Data_In;
  wire gpio_core_1_n_13;
  wire [6:0]gpio_io_i;
  wire [6:0]gpio_io_o;
  wire [6:0]gpio_io_t;
  wire gpio_xferAck_Reg;
  wire [6:0]ip2bus_data_i_D1;
  wire ip2bus_rdack_i;
  wire ip2bus_rdack_i_D1;
  wire ip2bus_wrack_i_D1;
  wire p_1_out;
  (* MAX_FANOUT = "10000" *) (* RTL_MAX_FANOUT = "found" *) (* sigis = "Clk" *) wire s_axi_aclk;
  wire [8:0]s_axi_araddr;
  (* MAX_FANOUT = "10000" *) (* RTL_MAX_FANOUT = "found" *) (* sigis = "Rst" *) wire s_axi_aresetn;
  wire s_axi_arready;
  wire s_axi_arvalid;
  wire [8:0]s_axi_awaddr;
  wire s_axi_awvalid;
  wire s_axi_bready;
  wire s_axi_bvalid;
  wire [6:0]\^s_axi_rdata ;
  wire s_axi_rready;
  wire s_axi_rvalid;
  wire [31:0]s_axi_wdata;
  wire s_axi_wready;
  wire s_axi_wvalid;

  assign ip2intc_irpt = \<const0> ;
  assign s_axi_awready = s_axi_wready;
  assign s_axi_bresp[1] = \<const0> ;
  assign s_axi_bresp[0] = \<const0> ;
  assign s_axi_rdata[31] = \<const0> ;
  assign s_axi_rdata[30] = \<const0> ;
  assign s_axi_rdata[29] = \<const0> ;
  assign s_axi_rdata[28] = \<const0> ;
  assign s_axi_rdata[27] = \<const0> ;
  assign s_axi_rdata[26] = \<const0> ;
  assign s_axi_rdata[25] = \<const0> ;
  assign s_axi_rdata[24] = \<const0> ;
  assign s_axi_rdata[23] = \<const0> ;
  assign s_axi_rdata[22] = \<const0> ;
  assign s_axi_rdata[21] = \<const0> ;
  assign s_axi_rdata[20] = \<const0> ;
  assign s_axi_rdata[19] = \<const0> ;
  assign s_axi_rdata[18] = \<const0> ;
  assign s_axi_rdata[17] = \<const0> ;
  assign s_axi_rdata[16] = \<const0> ;
  assign s_axi_rdata[15] = \<const0> ;
  assign s_axi_rdata[14] = \<const0> ;
  assign s_axi_rdata[13] = \<const0> ;
  assign s_axi_rdata[12] = \<const0> ;
  assign s_axi_rdata[11] = \<const0> ;
  assign s_axi_rdata[10] = \<const0> ;
  assign s_axi_rdata[9] = \<const0> ;
  assign s_axi_rdata[8] = \<const0> ;
  assign s_axi_rdata[7] = \<const0> ;
  assign s_axi_rdata[6:0] = \^s_axi_rdata [6:0];
  assign s_axi_rresp[1] = \<const0> ;
  assign s_axi_rresp[0] = \<const0> ;
  design_1_axi_gpio_0_1_axi_lite_ipif AXI_LITE_IPIF_I
       (.D({p_1_out,AXI_LITE_IPIF_I_n_10,AXI_LITE_IPIF_I_n_11,AXI_LITE_IPIF_I_n_12,AXI_LITE_IPIF_I_n_13,AXI_LITE_IPIF_I_n_14,AXI_LITE_IPIF_I_n_15}),
        .\Dual.gpio2_Data_Out_reg[0] (AXI_LITE_IPIF_I_n_34),
        .\Dual.gpio2_OE_reg[0] (AXI_LITE_IPIF_I_n_7),
        .\Dual.gpio2_OE_reg[0]_0 (AXI_LITE_IPIF_I_n_33),
        .\Dual.gpio_Data_In_reg[0] ({gpio_Data_In[0],gpio_Data_In[1],gpio_Data_In[2],gpio_Data_In[3],gpio_Data_In[4],gpio_Data_In[5],gpio_Data_In[6]}),
        .\Dual.gpio_Data_Out_reg[0] (AXI_LITE_IPIF_I_n_31),
        .E(AXI_LITE_IPIF_I_n_30),
        .GPIO2_DBus_i(GPIO2_DBus_i),
        .GPIO_DBus_i({GPIO_DBus_i[25],GPIO_DBus_i[26],GPIO_DBus_i[27],GPIO_DBus_i[28],GPIO_DBus_i[29],GPIO_DBus_i[30],GPIO_DBus_i[31]}),
        .GPIO_xferAck_i(GPIO_xferAck_i),
        .Q(bus2ip_addr),
        .Read_Reg_In(Read_Reg_In),
        .Read_Reg_Rst(Read_Reg_Rst),
        .bus2ip_cs(bus2ip_cs),
        .bus2ip_reset(bus2ip_reset),
        .bus2ip_rnw(bus2ip_rnw),
        .gpio2_io_o(gpio2_io_o),
        .gpio2_io_t(gpio2_io_t),
        .gpio_io_t(gpio_io_t),
        .gpio_xferAck_Reg(gpio_xferAck_Reg),
        .\ip2bus_data_i_D1_reg[25] (GPIO_DBus),
        .\ip2bus_data_i_D1_reg[25]_0 (ip2bus_data_i_D1),
        .ip2bus_rdack_i_D1(ip2bus_rdack_i_D1),
        .ip2bus_wrack_i_D1(ip2bus_wrack_i_D1),
        .s_axi_aclk(s_axi_aclk),
        .s_axi_araddr({s_axi_araddr[8],s_axi_araddr[3:2]}),
        .s_axi_aresetn(s_axi_aresetn),
        .s_axi_arready(s_axi_arready),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_awaddr({s_axi_awaddr[8],s_axi_awaddr[3:2]}),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_rdata(\^s_axi_rdata ),
        .s_axi_rready(s_axi_rready),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_wdata(s_axi_wdata[6:0]),
        .s_axi_wready(s_axi_wready),
        .s_axi_wvalid(s_axi_wvalid));
  GND GND
       (.G(\<const0> ));
  design_1_axi_gpio_0_1_GPIO_Core gpio_core_1
       (.D({p_1_out,AXI_LITE_IPIF_I_n_10,AXI_LITE_IPIF_I_n_11,AXI_LITE_IPIF_I_n_12,AXI_LITE_IPIF_I_n_13,AXI_LITE_IPIF_I_n_14,AXI_LITE_IPIF_I_n_15}),
        .\Dual.READ_REG_GEN[0].GPIO_DBus_i_reg[25]_0 ({gpio_Data_In[0],gpio_Data_In[1],gpio_Data_In[2],gpio_Data_In[3],gpio_Data_In[4],gpio_Data_In[5],gpio_Data_In[6]}),
        .E(AXI_LITE_IPIF_I_n_31),
        .GPIO2_DBus_i(GPIO2_DBus_i),
        .GPIO_DBus_i({GPIO_DBus_i[25],GPIO_DBus_i[26],GPIO_DBus_i[27],GPIO_DBus_i[28],GPIO_DBus_i[29],GPIO_DBus_i[30],GPIO_DBus_i[31]}),
        .GPIO_xferAck_i(GPIO_xferAck_i),
        .Q(bus2ip_addr),
        .Read_Reg_In(Read_Reg_In),
        .Read_Reg_Rst(Read_Reg_Rst),
        .\bus2ip_addr_i_reg[2] (AXI_LITE_IPIF_I_n_34),
        .\bus2ip_addr_i_reg[3] (AXI_LITE_IPIF_I_n_7),
        .bus2ip_cs(bus2ip_cs),
        .bus2ip_reset(bus2ip_reset),
        .bus2ip_rnw(bus2ip_rnw),
        .bus2ip_rnw_i_reg(AXI_LITE_IPIF_I_n_33),
        .bus2ip_rnw_i_reg_0(AXI_LITE_IPIF_I_n_30),
        .gpio2_io_i(gpio2_io_i),
        .gpio2_io_o(gpio2_io_o),
        .gpio2_io_t(gpio2_io_t),
        .gpio_io_i(gpio_io_i),
        .gpio_io_o(gpio_io_o),
        .gpio_io_t(gpio_io_t),
        .gpio_xferAck_Reg(gpio_xferAck_Reg),
        .ip2bus_rdack_i(ip2bus_rdack_i),
        .ip2bus_wrack_i_D1_reg(gpio_core_1_n_13),
        .s_axi_aclk(s_axi_aclk));
  FDRE \ip2bus_data_i_D1_reg[25] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(GPIO_DBus[6]),
        .Q(ip2bus_data_i_D1[6]),
        .R(bus2ip_reset));
  FDRE \ip2bus_data_i_D1_reg[26] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(GPIO_DBus[5]),
        .Q(ip2bus_data_i_D1[5]),
        .R(bus2ip_reset));
  FDRE \ip2bus_data_i_D1_reg[27] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(GPIO_DBus[4]),
        .Q(ip2bus_data_i_D1[4]),
        .R(bus2ip_reset));
  FDRE \ip2bus_data_i_D1_reg[28] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(GPIO_DBus[3]),
        .Q(ip2bus_data_i_D1[3]),
        .R(bus2ip_reset));
  FDRE \ip2bus_data_i_D1_reg[29] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(GPIO_DBus[2]),
        .Q(ip2bus_data_i_D1[2]),
        .R(bus2ip_reset));
  FDRE \ip2bus_data_i_D1_reg[30] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(GPIO_DBus[1]),
        .Q(ip2bus_data_i_D1[1]),
        .R(bus2ip_reset));
  FDRE \ip2bus_data_i_D1_reg[31] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(GPIO_DBus[0]),
        .Q(ip2bus_data_i_D1[0]),
        .R(bus2ip_reset));
  FDRE ip2bus_rdack_i_D1_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(ip2bus_rdack_i),
        .Q(ip2bus_rdack_i_D1),
        .R(bus2ip_reset));
  FDRE ip2bus_wrack_i_D1_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_core_1_n_13),
        .Q(ip2bus_wrack_i_D1),
        .R(bus2ip_reset));
endmodule

module design_1_axi_gpio_0_1_axi_lite_ipif
   (bus2ip_reset,
    bus2ip_rnw,
    bus2ip_cs,
    s_axi_rvalid,
    s_axi_bvalid,
    s_axi_arready,
    s_axi_wready,
    \Dual.gpio2_OE_reg[0] ,
    Q,
    D,
    \ip2bus_data_i_D1_reg[25] ,
    Read_Reg_In,
    E,
    \Dual.gpio_Data_Out_reg[0] ,
    Read_Reg_Rst,
    \Dual.gpio2_OE_reg[0]_0 ,
    \Dual.gpio2_Data_Out_reg[0] ,
    s_axi_rdata,
    s_axi_aclk,
    s_axi_arvalid,
    ip2bus_rdack_i_D1,
    ip2bus_wrack_i_D1,
    s_axi_bready,
    s_axi_rready,
    s_axi_awaddr,
    s_axi_araddr,
    s_axi_awvalid,
    s_axi_wvalid,
    s_axi_wdata,
    GPIO2_DBus_i,
    GPIO_DBus_i,
    gpio_io_t,
    \Dual.gpio_Data_In_reg[0] ,
    s_axi_aresetn,
    GPIO_xferAck_i,
    gpio_xferAck_Reg,
    gpio2_io_t,
    gpio2_io_o,
    \ip2bus_data_i_D1_reg[25]_0 );
  output bus2ip_reset;
  output bus2ip_rnw;
  output bus2ip_cs;
  output s_axi_rvalid;
  output s_axi_bvalid;
  output s_axi_arready;
  output s_axi_wready;
  output \Dual.gpio2_OE_reg[0] ;
  output [0:0]Q;
  output [6:0]D;
  output [6:0]\ip2bus_data_i_D1_reg[25] ;
  output [0:6]Read_Reg_In;
  output [0:0]E;
  output [0:0]\Dual.gpio_Data_Out_reg[0] ;
  output Read_Reg_Rst;
  output \Dual.gpio2_OE_reg[0]_0 ;
  output \Dual.gpio2_Data_Out_reg[0] ;
  output [6:0]s_axi_rdata;
  input s_axi_aclk;
  input s_axi_arvalid;
  input ip2bus_rdack_i_D1;
  input ip2bus_wrack_i_D1;
  input s_axi_bready;
  input s_axi_rready;
  input [2:0]s_axi_awaddr;
  input [2:0]s_axi_araddr;
  input s_axi_awvalid;
  input s_axi_wvalid;
  input [6:0]s_axi_wdata;
  input [0:0]GPIO2_DBus_i;
  input [6:0]GPIO_DBus_i;
  input [6:0]gpio_io_t;
  input [6:0]\Dual.gpio_Data_In_reg[0] ;
  input s_axi_aresetn;
  input GPIO_xferAck_i;
  input gpio_xferAck_Reg;
  input [0:0]gpio2_io_t;
  input [0:0]gpio2_io_o;
  input [6:0]\ip2bus_data_i_D1_reg[25]_0 ;

  wire [6:0]D;
  wire \Dual.gpio2_Data_Out_reg[0] ;
  wire \Dual.gpio2_OE_reg[0] ;
  wire \Dual.gpio2_OE_reg[0]_0 ;
  wire [6:0]\Dual.gpio_Data_In_reg[0] ;
  wire [0:0]\Dual.gpio_Data_Out_reg[0] ;
  wire [0:0]E;
  wire [0:0]GPIO2_DBus_i;
  wire [6:0]GPIO_DBus_i;
  wire GPIO_xferAck_i;
  wire [0:0]Q;
  wire [0:6]Read_Reg_In;
  wire Read_Reg_Rst;
  wire bus2ip_cs;
  wire bus2ip_reset;
  wire bus2ip_rnw;
  wire [0:0]gpio2_io_o;
  wire [0:0]gpio2_io_t;
  wire [6:0]gpio_io_t;
  wire gpio_xferAck_Reg;
  wire [6:0]\ip2bus_data_i_D1_reg[25] ;
  wire [6:0]\ip2bus_data_i_D1_reg[25]_0 ;
  wire ip2bus_rdack_i_D1;
  wire ip2bus_wrack_i_D1;
  wire s_axi_aclk;
  wire [2:0]s_axi_araddr;
  wire s_axi_aresetn;
  wire s_axi_arready;
  wire s_axi_arvalid;
  wire [2:0]s_axi_awaddr;
  wire s_axi_awvalid;
  wire s_axi_bready;
  wire s_axi_bvalid;
  wire [6:0]s_axi_rdata;
  wire s_axi_rready;
  wire s_axi_rvalid;
  wire [6:0]s_axi_wdata;
  wire s_axi_wready;
  wire s_axi_wvalid;

  design_1_axi_gpio_0_1_slave_attachment I_SLAVE_ATTACHMENT
       (.D(D),
        .\Dual.gpio2_Data_Out_reg[0] (\Dual.gpio2_Data_Out_reg[0] ),
        .\Dual.gpio2_OE_reg[0] (bus2ip_rnw),
        .\Dual.gpio2_OE_reg[0]_0 (\Dual.gpio2_OE_reg[0] ),
        .\Dual.gpio2_OE_reg[0]_1 (\Dual.gpio2_OE_reg[0]_0 ),
        .\Dual.gpio_Data_In_reg[0] (\Dual.gpio_Data_In_reg[0] ),
        .\Dual.gpio_Data_Out_reg[0] (\Dual.gpio_Data_Out_reg[0] ),
        .E(E),
        .GPIO2_DBus_i(GPIO2_DBus_i),
        .GPIO_DBus_i(GPIO_DBus_i),
        .GPIO_xferAck_i(GPIO_xferAck_i),
        .\MEM_DECODE_GEN[0].cs_out_i_reg[0] (bus2ip_cs),
        .Q(Q),
        .Read_Reg_In(Read_Reg_In),
        .Read_Reg_Rst(Read_Reg_Rst),
        .bus2ip_rnw_i_reg_0(bus2ip_reset),
        .gpio2_io_o(gpio2_io_o),
        .gpio2_io_t(gpio2_io_t),
        .gpio_io_t(gpio_io_t),
        .gpio_xferAck_Reg(gpio_xferAck_Reg),
        .\ip2bus_data_i_D1_reg[25] (\ip2bus_data_i_D1_reg[25] ),
        .\ip2bus_data_i_D1_reg[25]_0 (\ip2bus_data_i_D1_reg[25]_0 ),
        .ip2bus_rdack_i_D1(ip2bus_rdack_i_D1),
        .ip2bus_wrack_i_D1(ip2bus_wrack_i_D1),
        .s_axi_aclk(s_axi_aclk),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_aresetn(s_axi_aresetn),
        .s_axi_arready(s_axi_arready),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rready(s_axi_rready),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wready(s_axi_wready),
        .s_axi_wvalid(s_axi_wvalid));
endmodule

module design_1_axi_gpio_0_1_cdc_sync
   (scndry_vect_out,
    gpio_io_i,
    s_axi_aclk);
  output [6:0]scndry_vect_out;
  input [6:0]gpio_io_i;
  input s_axi_aclk;

  wire [6:0]gpio_io_i;
  wire s_axi_aclk;
  wire s_level_out_bus_d1_cdc_to_0;
  wire s_level_out_bus_d1_cdc_to_1;
  wire s_level_out_bus_d1_cdc_to_2;
  wire s_level_out_bus_d1_cdc_to_3;
  wire s_level_out_bus_d1_cdc_to_4;
  wire s_level_out_bus_d1_cdc_to_5;
  wire s_level_out_bus_d1_cdc_to_6;
  wire s_level_out_bus_d2_0;
  wire s_level_out_bus_d2_1;
  wire s_level_out_bus_d2_2;
  wire s_level_out_bus_d2_3;
  wire s_level_out_bus_d2_4;
  wire s_level_out_bus_d2_5;
  wire s_level_out_bus_d2_6;
  wire s_level_out_bus_d3_0;
  wire s_level_out_bus_d3_1;
  wire s_level_out_bus_d3_2;
  wire s_level_out_bus_d3_3;
  wire s_level_out_bus_d3_4;
  wire s_level_out_bus_d3_5;
  wire s_level_out_bus_d3_6;
  wire [6:0]scndry_vect_out;

  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d1_cdc_to_0),
        .Q(s_level_out_bus_d2_0),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[1].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d1_cdc_to_1),
        .Q(s_level_out_bus_d2_1),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[2].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d1_cdc_to_2),
        .Q(s_level_out_bus_d2_2),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[3].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d1_cdc_to_3),
        .Q(s_level_out_bus_d2_3),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[4].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d1_cdc_to_4),
        .Q(s_level_out_bus_d2_4),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[5].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d1_cdc_to_5),
        .Q(s_level_out_bus_d2_5),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[6].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d1_cdc_to_6),
        .Q(s_level_out_bus_d2_6),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d2_0),
        .Q(s_level_out_bus_d3_0),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[1].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d2_1),
        .Q(s_level_out_bus_d3_1),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[2].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d2_2),
        .Q(s_level_out_bus_d3_2),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[3].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d2_3),
        .Q(s_level_out_bus_d3_3),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[4].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d2_4),
        .Q(s_level_out_bus_d3_4),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[5].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d2_5),
        .Q(s_level_out_bus_d3_5),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[6].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d2_6),
        .Q(s_level_out_bus_d3_6),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d4[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d4 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d3_0),
        .Q(scndry_vect_out[0]),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d4[1].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d4 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d3_1),
        .Q(scndry_vect_out[1]),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d4[2].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d4 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d3_2),
        .Q(scndry_vect_out[2]),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d4[3].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d4 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d3_3),
        .Q(scndry_vect_out[3]),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d4[4].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d4 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d3_4),
        .Q(scndry_vect_out[4]),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d4[5].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d4 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d3_5),
        .Q(scndry_vect_out[5]),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d4[6].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d4 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_level_out_bus_d3_6),
        .Q(scndry_vect_out[6]),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_IN_cdc_to[0].CROSS2_PLEVEL_IN2SCNDRY_IN_cdc_to 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i[0]),
        .Q(s_level_out_bus_d1_cdc_to_0),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_IN_cdc_to[1].CROSS2_PLEVEL_IN2SCNDRY_IN_cdc_to 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i[1]),
        .Q(s_level_out_bus_d1_cdc_to_1),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_IN_cdc_to[2].CROSS2_PLEVEL_IN2SCNDRY_IN_cdc_to 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i[2]),
        .Q(s_level_out_bus_d1_cdc_to_2),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_IN_cdc_to[3].CROSS2_PLEVEL_IN2SCNDRY_IN_cdc_to 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i[3]),
        .Q(s_level_out_bus_d1_cdc_to_3),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_IN_cdc_to[4].CROSS2_PLEVEL_IN2SCNDRY_IN_cdc_to 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i[4]),
        .Q(s_level_out_bus_d1_cdc_to_4),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_IN_cdc_to[5].CROSS2_PLEVEL_IN2SCNDRY_IN_cdc_to 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i[5]),
        .Q(s_level_out_bus_d1_cdc_to_5),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_IN_cdc_to[6].CROSS2_PLEVEL_IN2SCNDRY_IN_cdc_to 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio_io_i[6]),
        .Q(s_level_out_bus_d1_cdc_to_6),
        .R(1'b0));
endmodule

(* ORIG_REF_NAME = "cdc_sync" *) 
module design_1_axi_gpio_0_1_cdc_sync__parameterized0
   (scndry_vect_out,
    gpio2_io_i,
    s_axi_aclk);
  output [0:0]scndry_vect_out;
  input [0:0]gpio2_io_i;
  input s_axi_aclk;

  wire \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2_n_0 ;
  wire \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3_n_0 ;
  wire Q;
  wire [0:0]gpio2_io_i;
  wire s_axi_aclk;
  wire [0:0]scndry_vect_out;

  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(Q),
        .Q(\GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2_n_0 ),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(\GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d2[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d2_n_0 ),
        .Q(\GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3_n_0 ),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d4[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d4 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(\GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_CROSS_PLEVEL_IN2SCNDRY_bus_d3[0].CROSS2_PLEVEL_IN2SCNDRY_s_level_out_bus_d3_n_0 ),
        .Q(scndry_vect_out),
        .R(1'b0));
  (* ASYNC_REG *) 
  (* XILINX_LEGACY_PRIM = "FDR" *) 
  (* box_type = "PRIMITIVE" *) 
  FDRE #(
    .INIT(1'b0)) 
    \GENERATE_LEVEL_P_S_CDC.MULTI_BIT.FOR_IN_cdc_to[0].CROSS2_PLEVEL_IN2SCNDRY_IN_cdc_to 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(gpio2_io_i),
        .Q(Q),
        .R(1'b0));
endmodule

(* CHECK_LICENSE_TYPE = "design_1_axi_gpio_0_1,axi_gpio,{}" *) (* downgradeipidentifiedwarnings = "yes" *) (* x_core_info = "axi_gpio,Vivado 2016.4" *) 
(* NotValidForBitStream *)
module design_1_axi_gpio_0_1
   (s_axi_aclk,
    s_axi_aresetn,
    s_axi_awaddr,
    s_axi_awvalid,
    s_axi_awready,
    s_axi_wdata,
    s_axi_wstrb,
    s_axi_wvalid,
    s_axi_wready,
    s_axi_bresp,
    s_axi_bvalid,
    s_axi_bready,
    s_axi_araddr,
    s_axi_arvalid,
    s_axi_arready,
    s_axi_rdata,
    s_axi_rresp,
    s_axi_rvalid,
    s_axi_rready,
    gpio_io_o,
    gpio2_io_i);
  (* x_interface_info = "xilinx.com:signal:clock:1.0 S_AXI_ACLK CLK" *) input s_axi_aclk;
  (* x_interface_info = "xilinx.com:signal:reset:1.0 S_AXI_ARESETN RST" *) input s_axi_aresetn;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *) input [8:0]s_axi_awaddr;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *) input s_axi_awvalid;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *) output s_axi_awready;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *) input [31:0]s_axi_wdata;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *) input [3:0]s_axi_wstrb;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *) input s_axi_wvalid;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *) output s_axi_wready;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *) output [1:0]s_axi_bresp;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *) output s_axi_bvalid;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *) input s_axi_bready;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *) input [8:0]s_axi_araddr;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *) input s_axi_arvalid;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *) output s_axi_arready;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *) output [31:0]s_axi_rdata;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *) output [1:0]s_axi_rresp;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *) output s_axi_rvalid;
  (* x_interface_info = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *) input s_axi_rready;
  (* x_interface_info = "xilinx.com:interface:gpio:1.0 GPIO TRI_O" *) output [6:0]gpio_io_o;
  (* x_interface_info = "xilinx.com:interface:gpio:1.0 GPIO2 TRI_I" *) input [0:0]gpio2_io_i;

  wire [0:0]gpio2_io_i;
  wire [6:0]gpio_io_o;
  wire s_axi_aclk;
  wire [8:0]s_axi_araddr;
  wire s_axi_aresetn;
  wire s_axi_arready;
  wire s_axi_arvalid;
  wire [8:0]s_axi_awaddr;
  wire s_axi_awready;
  wire s_axi_awvalid;
  wire s_axi_bready;
  wire [1:0]s_axi_bresp;
  wire s_axi_bvalid;
  wire [31:0]s_axi_rdata;
  wire s_axi_rready;
  wire [1:0]s_axi_rresp;
  wire s_axi_rvalid;
  wire [31:0]s_axi_wdata;
  wire s_axi_wready;
  wire [3:0]s_axi_wstrb;
  wire s_axi_wvalid;
  wire NLW_U0_ip2intc_irpt_UNCONNECTED;
  wire [0:0]NLW_U0_gpio2_io_o_UNCONNECTED;
  wire [0:0]NLW_U0_gpio2_io_t_UNCONNECTED;
  wire [6:0]NLW_U0_gpio_io_t_UNCONNECTED;

  (* C_ALL_INPUTS = "0" *) 
  (* C_ALL_INPUTS_2 = "1" *) 
  (* C_ALL_OUTPUTS = "1" *) 
  (* C_ALL_OUTPUTS_2 = "0" *) 
  (* C_DOUT_DEFAULT = "0" *) 
  (* C_DOUT_DEFAULT_2 = "0" *) 
  (* C_FAMILY = "zynq" *) 
  (* C_GPIO2_WIDTH = "1" *) 
  (* C_GPIO_WIDTH = "7" *) 
  (* C_INTERRUPT_PRESENT = "0" *) 
  (* C_IS_DUAL = "1" *) 
  (* C_S_AXI_ADDR_WIDTH = "9" *) 
  (* C_S_AXI_DATA_WIDTH = "32" *) 
  (* C_TRI_DEFAULT = "-1" *) 
  (* C_TRI_DEFAULT_2 = "-1" *) 
  (* downgradeipidentifiedwarnings = "yes" *) 
  (* ip_group = "LOGICORE" *) 
  design_1_axi_gpio_0_1_axi_gpio U0
       (.gpio2_io_i(gpio2_io_i),
        .gpio2_io_o(NLW_U0_gpio2_io_o_UNCONNECTED[0]),
        .gpio2_io_t(NLW_U0_gpio2_io_t_UNCONNECTED[0]),
        .gpio_io_i({1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0}),
        .gpio_io_o(gpio_io_o),
        .gpio_io_t(NLW_U0_gpio_io_t_UNCONNECTED[6:0]),
        .ip2intc_irpt(NLW_U0_ip2intc_irpt_UNCONNECTED),
        .s_axi_aclk(s_axi_aclk),
        .s_axi_araddr(s_axi_araddr),
        .s_axi_aresetn(s_axi_aresetn),
        .s_axi_arready(s_axi_arready),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_awaddr(s_axi_awaddr),
        .s_axi_awready(s_axi_awready),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid),
        .s_axi_rdata(s_axi_rdata),
        .s_axi_rready(s_axi_rready),
        .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wready(s_axi_wready),
        .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wvalid(s_axi_wvalid));
endmodule

module design_1_axi_gpio_0_1_slave_attachment
   (bus2ip_rnw_i_reg_0,
    \Dual.gpio2_OE_reg[0] ,
    \MEM_DECODE_GEN[0].cs_out_i_reg[0] ,
    s_axi_rvalid,
    s_axi_bvalid,
    s_axi_arready,
    s_axi_wready,
    \Dual.gpio2_OE_reg[0]_0 ,
    D,
    \ip2bus_data_i_D1_reg[25] ,
    Read_Reg_In,
    Q,
    E,
    \Dual.gpio_Data_Out_reg[0] ,
    Read_Reg_Rst,
    \Dual.gpio2_OE_reg[0]_1 ,
    \Dual.gpio2_Data_Out_reg[0] ,
    s_axi_rdata,
    s_axi_aclk,
    s_axi_arvalid,
    ip2bus_rdack_i_D1,
    ip2bus_wrack_i_D1,
    s_axi_bready,
    s_axi_rready,
    s_axi_awaddr,
    s_axi_araddr,
    s_axi_awvalid,
    s_axi_wvalid,
    s_axi_wdata,
    GPIO2_DBus_i,
    GPIO_DBus_i,
    gpio_io_t,
    \Dual.gpio_Data_In_reg[0] ,
    s_axi_aresetn,
    GPIO_xferAck_i,
    gpio_xferAck_Reg,
    gpio2_io_t,
    gpio2_io_o,
    \ip2bus_data_i_D1_reg[25]_0 );
  output bus2ip_rnw_i_reg_0;
  output \Dual.gpio2_OE_reg[0] ;
  output \MEM_DECODE_GEN[0].cs_out_i_reg[0] ;
  output s_axi_rvalid;
  output s_axi_bvalid;
  output s_axi_arready;
  output s_axi_wready;
  output \Dual.gpio2_OE_reg[0]_0 ;
  output [6:0]D;
  output [6:0]\ip2bus_data_i_D1_reg[25] ;
  output [0:6]Read_Reg_In;
  output [0:0]Q;
  output [0:0]E;
  output [0:0]\Dual.gpio_Data_Out_reg[0] ;
  output Read_Reg_Rst;
  output \Dual.gpio2_OE_reg[0]_1 ;
  output \Dual.gpio2_Data_Out_reg[0] ;
  output [6:0]s_axi_rdata;
  input s_axi_aclk;
  input s_axi_arvalid;
  input ip2bus_rdack_i_D1;
  input ip2bus_wrack_i_D1;
  input s_axi_bready;
  input s_axi_rready;
  input [2:0]s_axi_awaddr;
  input [2:0]s_axi_araddr;
  input s_axi_awvalid;
  input s_axi_wvalid;
  input [6:0]s_axi_wdata;
  input [0:0]GPIO2_DBus_i;
  input [6:0]GPIO_DBus_i;
  input [6:0]gpio_io_t;
  input [6:0]\Dual.gpio_Data_In_reg[0] ;
  input s_axi_aresetn;
  input GPIO_xferAck_i;
  input gpio_xferAck_Reg;
  input [0:0]gpio2_io_t;
  input [0:0]gpio2_io_o;
  input [6:0]\ip2bus_data_i_D1_reg[25]_0 ;

  wire [6:0]D;
  wire \Dual.gpio2_Data_Out_reg[0] ;
  wire \Dual.gpio2_OE_reg[0] ;
  wire \Dual.gpio2_OE_reg[0]_0 ;
  wire \Dual.gpio2_OE_reg[0]_1 ;
  wire [6:0]\Dual.gpio_Data_In_reg[0] ;
  wire [0:0]\Dual.gpio_Data_Out_reg[0] ;
  wire [0:0]E;
  wire [0:0]GPIO2_DBus_i;
  wire [6:0]GPIO_DBus_i;
  wire GPIO_xferAck_i;
  wire [3:0]\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 ;
  wire \MEM_DECODE_GEN[0].cs_out_i_reg[0] ;
  wire [0:0]Q;
  wire [0:6]Read_Reg_In;
  wire Read_Reg_Rst;
  wire [0:5]bus2ip_addr;
  wire \bus2ip_addr_i[2]_i_1_n_0 ;
  wire \bus2ip_addr_i[3]_i_1_n_0 ;
  wire \bus2ip_addr_i[8]_i_1_n_0 ;
  wire bus2ip_rnw_i06_out;
  wire bus2ip_rnw_i_reg_0;
  wire clear;
  wire [0:0]gpio2_io_o;
  wire [0:0]gpio2_io_t;
  wire [6:0]gpio_io_t;
  wire gpio_xferAck_Reg;
  wire [6:0]\ip2bus_data_i_D1_reg[25] ;
  wire [6:0]\ip2bus_data_i_D1_reg[25]_0 ;
  wire ip2bus_rdack_i_D1;
  wire ip2bus_wrack_i_D1;
  wire is_read;
  wire is_read_i_1_n_0;
  wire is_write;
  wire is_write_i_1_n_0;
  wire is_write_reg_n_0;
  wire [1:0]p_0_out;
  wire p_1_in;
  wire [3:0]plusOp;
  wire s_axi_aclk;
  wire [2:0]s_axi_araddr;
  wire s_axi_aresetn;
  wire s_axi_arready;
  wire s_axi_arvalid;
  wire [2:0]s_axi_awaddr;
  wire s_axi_awvalid;
  wire s_axi_bready;
  wire s_axi_bvalid;
  wire s_axi_bvalid_i_i_1_n_0;
  wire [6:0]s_axi_rdata;
  wire s_axi_rdata_i;
  wire s_axi_rready;
  wire s_axi_rvalid;
  wire s_axi_rvalid_i_i_1_n_0;
  wire [6:0]s_axi_wdata;
  wire s_axi_wready;
  wire s_axi_wvalid;
  wire start2;
  wire start2_i_1_n_0;
  wire [1:0]state;
  wire \state[1]_i_2_n_0 ;
  wire \state[1]_i_3_n_0 ;

  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT1 #(
    .INIT(2'h1)) 
    \INCLUDE_DPHASE_TIMER.dpto_cnt[0]_i_1 
       (.I0(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [0]),
        .O(plusOp[0]));
  (* SOFT_HLUTNM = "soft_lutpair6" *) 
  LUT2 #(
    .INIT(4'h6)) 
    \INCLUDE_DPHASE_TIMER.dpto_cnt[1]_i_1 
       (.I0(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [0]),
        .I1(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [1]),
        .O(plusOp[1]));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT3 #(
    .INIT(8'h78)) 
    \INCLUDE_DPHASE_TIMER.dpto_cnt[2]_i_1 
       (.I0(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [1]),
        .I1(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [0]),
        .I2(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [2]),
        .O(plusOp[2]));
  LUT2 #(
    .INIT(4'h9)) 
    \INCLUDE_DPHASE_TIMER.dpto_cnt[3]_i_1 
       (.I0(state[1]),
        .I1(state[0]),
        .O(clear));
  (* SOFT_HLUTNM = "soft_lutpair5" *) 
  LUT4 #(
    .INIT(16'h7F80)) 
    \INCLUDE_DPHASE_TIMER.dpto_cnt[3]_i_2 
       (.I0(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [2]),
        .I1(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [0]),
        .I2(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [1]),
        .I3(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [3]),
        .O(plusOp[3]));
  FDRE \INCLUDE_DPHASE_TIMER.dpto_cnt_reg[0] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(plusOp[0]),
        .Q(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [0]),
        .R(clear));
  FDRE \INCLUDE_DPHASE_TIMER.dpto_cnt_reg[1] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(plusOp[1]),
        .Q(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [1]),
        .R(clear));
  FDRE \INCLUDE_DPHASE_TIMER.dpto_cnt_reg[2] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(plusOp[2]),
        .Q(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [2]),
        .R(clear));
  FDRE \INCLUDE_DPHASE_TIMER.dpto_cnt_reg[3] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(plusOp[3]),
        .Q(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 [3]),
        .R(clear));
  design_1_axi_gpio_0_1_address_decoder I_DECODER
       (.D(D),
        .\Dual.gpio2_Data_Out_reg[0] (\Dual.gpio2_Data_Out_reg[0] ),
        .\Dual.gpio2_OE_reg[0] (\Dual.gpio2_OE_reg[0]_0 ),
        .\Dual.gpio2_OE_reg[0]_0 (\Dual.gpio2_OE_reg[0]_1 ),
        .\Dual.gpio_Data_In_reg[0] (\Dual.gpio_Data_In_reg[0] ),
        .\Dual.gpio_Data_Out_reg[0] (\Dual.gpio_Data_Out_reg[0] ),
        .E(E),
        .GPIO2_DBus_i(GPIO2_DBus_i),
        .GPIO_DBus_i(GPIO_DBus_i),
        .GPIO_xferAck_i(GPIO_xferAck_i),
        .\MEM_DECODE_GEN[0].cs_out_i_reg[0]_0 (\MEM_DECODE_GEN[0].cs_out_i_reg[0] ),
        .Q(\INCLUDE_DPHASE_TIMER.dpto_cnt_reg__0 ),
        .Read_Reg_In(Read_Reg_In),
        .Read_Reg_Rst(Read_Reg_Rst),
        .\bus2ip_addr_i_reg[8] ({bus2ip_addr[0],bus2ip_addr[5],Q}),
        .bus2ip_rnw_i_reg(\Dual.gpio2_OE_reg[0] ),
        .gpio2_io_o(gpio2_io_o),
        .gpio2_io_t(gpio2_io_t),
        .gpio_io_t(gpio_io_t),
        .gpio_xferAck_Reg(gpio_xferAck_Reg),
        .\ip2bus_data_i_D1_reg[25] (\ip2bus_data_i_D1_reg[25] ),
        .ip2bus_rdack_i_D1(ip2bus_rdack_i_D1),
        .ip2bus_wrack_i_D1(ip2bus_wrack_i_D1),
        .is_read(is_read),
        .is_write_reg(is_write_reg_n_0),
        .rst_reg(bus2ip_rnw_i_reg_0),
        .s_axi_aclk(s_axi_aclk),
        .s_axi_aresetn(s_axi_aresetn),
        .s_axi_arready(s_axi_arready),
        .s_axi_wdata(s_axi_wdata),
        .s_axi_wready(s_axi_wready),
        .start2(start2));
  LUT5 #(
    .INIT(32'hABAAA8AA)) 
    \bus2ip_addr_i[2]_i_1 
       (.I0(s_axi_awaddr[0]),
        .I1(state[1]),
        .I2(state[0]),
        .I3(s_axi_arvalid),
        .I4(s_axi_araddr[0]),
        .O(\bus2ip_addr_i[2]_i_1_n_0 ));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT5 #(
    .INIT(32'hABAAA8AA)) 
    \bus2ip_addr_i[3]_i_1 
       (.I0(s_axi_awaddr[1]),
        .I1(state[1]),
        .I2(state[0]),
        .I3(s_axi_arvalid),
        .I4(s_axi_araddr[1]),
        .O(\bus2ip_addr_i[3]_i_1_n_0 ));
  LUT5 #(
    .INIT(32'hABAAA8AA)) 
    \bus2ip_addr_i[8]_i_1 
       (.I0(s_axi_awaddr[2]),
        .I1(state[1]),
        .I2(state[0]),
        .I3(s_axi_arvalid),
        .I4(s_axi_araddr[2]),
        .O(\bus2ip_addr_i[8]_i_1_n_0 ));
  FDRE \bus2ip_addr_i_reg[2] 
       (.C(s_axi_aclk),
        .CE(start2_i_1_n_0),
        .D(\bus2ip_addr_i[2]_i_1_n_0 ),
        .Q(Q),
        .R(bus2ip_rnw_i_reg_0));
  FDRE \bus2ip_addr_i_reg[3] 
       (.C(s_axi_aclk),
        .CE(start2_i_1_n_0),
        .D(\bus2ip_addr_i[3]_i_1_n_0 ),
        .Q(bus2ip_addr[5]),
        .R(bus2ip_rnw_i_reg_0));
  FDRE \bus2ip_addr_i_reg[8] 
       (.C(s_axi_aclk),
        .CE(start2_i_1_n_0),
        .D(\bus2ip_addr_i[8]_i_1_n_0 ),
        .Q(bus2ip_addr[0]),
        .R(bus2ip_rnw_i_reg_0));
  (* SOFT_HLUTNM = "soft_lutpair4" *) 
  LUT3 #(
    .INIT(8'h02)) 
    bus2ip_rnw_i_i_1
       (.I0(s_axi_arvalid),
        .I1(state[0]),
        .I2(state[1]),
        .O(bus2ip_rnw_i06_out));
  FDRE bus2ip_rnw_i_reg
       (.C(s_axi_aclk),
        .CE(start2_i_1_n_0),
        .D(bus2ip_rnw_i06_out),
        .Q(\Dual.gpio2_OE_reg[0] ),
        .R(bus2ip_rnw_i_reg_0));
  LUT5 #(
    .INIT(32'h3FFA000A)) 
    is_read_i_1
       (.I0(s_axi_arvalid),
        .I1(\state[1]_i_2_n_0 ),
        .I2(state[1]),
        .I3(state[0]),
        .I4(is_read),
        .O(is_read_i_1_n_0));
  FDRE is_read_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(is_read_i_1_n_0),
        .Q(is_read),
        .R(bus2ip_rnw_i_reg_0));
  LUT6 #(
    .INIT(64'h1000FFFF10000000)) 
    is_write_i_1
       (.I0(state[1]),
        .I1(s_axi_arvalid),
        .I2(s_axi_wvalid),
        .I3(s_axi_awvalid),
        .I4(is_write),
        .I5(is_write_reg_n_0),
        .O(is_write_i_1_n_0));
  LUT6 #(
    .INIT(64'hF88800000000FFFF)) 
    is_write_i_2
       (.I0(s_axi_bready),
        .I1(s_axi_bvalid),
        .I2(s_axi_rready),
        .I3(s_axi_rvalid),
        .I4(state[1]),
        .I5(state[0]),
        .O(is_write));
  FDRE is_write_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(is_write_i_1_n_0),
        .Q(is_write_reg_n_0),
        .R(bus2ip_rnw_i_reg_0));
  LUT1 #(
    .INIT(2'h1)) 
    rst_i_1
       (.I0(s_axi_aresetn),
        .O(p_1_in));
  FDRE rst_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(p_1_in),
        .Q(bus2ip_rnw_i_reg_0),
        .R(1'b0));
  LUT5 #(
    .INIT(32'h08FF0808)) 
    s_axi_bvalid_i_i_1
       (.I0(s_axi_wready),
        .I1(state[1]),
        .I2(state[0]),
        .I3(s_axi_bready),
        .I4(s_axi_bvalid),
        .O(s_axi_bvalid_i_i_1_n_0));
  FDRE #(
    .INIT(1'b0)) 
    s_axi_bvalid_i_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_axi_bvalid_i_i_1_n_0),
        .Q(s_axi_bvalid),
        .R(bus2ip_rnw_i_reg_0));
  LUT2 #(
    .INIT(4'h2)) 
    \s_axi_rdata_i[6]_i_1 
       (.I0(state[0]),
        .I1(state[1]),
        .O(s_axi_rdata_i));
  FDRE #(
    .INIT(1'b0)) 
    \s_axi_rdata_i_reg[0] 
       (.C(s_axi_aclk),
        .CE(s_axi_rdata_i),
        .D(\ip2bus_data_i_D1_reg[25]_0 [0]),
        .Q(s_axi_rdata[0]),
        .R(bus2ip_rnw_i_reg_0));
  FDRE #(
    .INIT(1'b0)) 
    \s_axi_rdata_i_reg[1] 
       (.C(s_axi_aclk),
        .CE(s_axi_rdata_i),
        .D(\ip2bus_data_i_D1_reg[25]_0 [1]),
        .Q(s_axi_rdata[1]),
        .R(bus2ip_rnw_i_reg_0));
  FDRE #(
    .INIT(1'b0)) 
    \s_axi_rdata_i_reg[2] 
       (.C(s_axi_aclk),
        .CE(s_axi_rdata_i),
        .D(\ip2bus_data_i_D1_reg[25]_0 [2]),
        .Q(s_axi_rdata[2]),
        .R(bus2ip_rnw_i_reg_0));
  FDRE #(
    .INIT(1'b0)) 
    \s_axi_rdata_i_reg[3] 
       (.C(s_axi_aclk),
        .CE(s_axi_rdata_i),
        .D(\ip2bus_data_i_D1_reg[25]_0 [3]),
        .Q(s_axi_rdata[3]),
        .R(bus2ip_rnw_i_reg_0));
  FDRE #(
    .INIT(1'b0)) 
    \s_axi_rdata_i_reg[4] 
       (.C(s_axi_aclk),
        .CE(s_axi_rdata_i),
        .D(\ip2bus_data_i_D1_reg[25]_0 [4]),
        .Q(s_axi_rdata[4]),
        .R(bus2ip_rnw_i_reg_0));
  FDRE #(
    .INIT(1'b0)) 
    \s_axi_rdata_i_reg[5] 
       (.C(s_axi_aclk),
        .CE(s_axi_rdata_i),
        .D(\ip2bus_data_i_D1_reg[25]_0 [5]),
        .Q(s_axi_rdata[5]),
        .R(bus2ip_rnw_i_reg_0));
  FDRE #(
    .INIT(1'b0)) 
    \s_axi_rdata_i_reg[6] 
       (.C(s_axi_aclk),
        .CE(s_axi_rdata_i),
        .D(\ip2bus_data_i_D1_reg[25]_0 [6]),
        .Q(s_axi_rdata[6]),
        .R(bus2ip_rnw_i_reg_0));
  LUT5 #(
    .INIT(32'h08FF0808)) 
    s_axi_rvalid_i_i_1
       (.I0(s_axi_arready),
        .I1(state[0]),
        .I2(state[1]),
        .I3(s_axi_rready),
        .I4(s_axi_rvalid),
        .O(s_axi_rvalid_i_i_1_n_0));
  FDRE #(
    .INIT(1'b0)) 
    s_axi_rvalid_i_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(s_axi_rvalid_i_i_1_n_0),
        .Q(s_axi_rvalid),
        .R(bus2ip_rnw_i_reg_0));
  LUT5 #(
    .INIT(32'h000000F8)) 
    start2_i_1
       (.I0(s_axi_awvalid),
        .I1(s_axi_wvalid),
        .I2(s_axi_arvalid),
        .I3(state[0]),
        .I4(state[1]),
        .O(start2_i_1_n_0));
  FDRE start2_reg
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(start2_i_1_n_0),
        .Q(start2),
        .R(bus2ip_rnw_i_reg_0));
  LUT5 #(
    .INIT(32'h0FFFAACC)) 
    \state[0]_i_1 
       (.I0(s_axi_wready),
        .I1(s_axi_arvalid),
        .I2(\state[1]_i_2_n_0 ),
        .I3(state[1]),
        .I4(state[0]),
        .O(p_0_out[0]));
  LUT6 #(
    .INIT(64'h2E2E2E2ECCCCFFCC)) 
    \state[1]_i_1 
       (.I0(s_axi_arready),
        .I1(state[1]),
        .I2(\state[1]_i_2_n_0 ),
        .I3(\state[1]_i_3_n_0 ),
        .I4(s_axi_arvalid),
        .I5(state[0]),
        .O(p_0_out[1]));
  LUT4 #(
    .INIT(16'hF888)) 
    \state[1]_i_2 
       (.I0(s_axi_bready),
        .I1(s_axi_bvalid),
        .I2(s_axi_rready),
        .I3(s_axi_rvalid),
        .O(\state[1]_i_2_n_0 ));
  LUT2 #(
    .INIT(4'h8)) 
    \state[1]_i_3 
       (.I0(s_axi_awvalid),
        .I1(s_axi_wvalid),
        .O(\state[1]_i_3_n_0 ));
  FDRE \state_reg[0] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(p_0_out[0]),
        .Q(state[0]),
        .R(bus2ip_rnw_i_reg_0));
  FDRE \state_reg[1] 
       (.C(s_axi_aclk),
        .CE(1'b1),
        .D(p_0_out[1]),
        .Q(state[1]),
        .R(bus2ip_rnw_i_reg_0));
endmodule
`ifndef GLBL
`define GLBL
`timescale  1 ps / 1 ps

module glbl ();

    parameter ROC_WIDTH = 100000;
    parameter TOC_WIDTH = 0;

//--------   STARTUP Globals --------------
    wire GSR;
    wire GTS;
    wire GWE;
    wire PRLD;
    tri1 p_up_tmp;
    tri (weak1, strong0) PLL_LOCKG = p_up_tmp;

    wire PROGB_GLBL;
    wire CCLKO_GLBL;
    wire FCSBO_GLBL;
    wire [3:0] DO_GLBL;
    wire [3:0] DI_GLBL;
   
    reg GSR_int;
    reg GTS_int;
    reg PRLD_int;

//--------   JTAG Globals --------------
    wire JTAG_TDO_GLBL;
    wire JTAG_TCK_GLBL;
    wire JTAG_TDI_GLBL;
    wire JTAG_TMS_GLBL;
    wire JTAG_TRST_GLBL;

    reg JTAG_CAPTURE_GLBL;
    reg JTAG_RESET_GLBL;
    reg JTAG_SHIFT_GLBL;
    reg JTAG_UPDATE_GLBL;
    reg JTAG_RUNTEST_GLBL;

    reg JTAG_SEL1_GLBL = 0;
    reg JTAG_SEL2_GLBL = 0 ;
    reg JTAG_SEL3_GLBL = 0;
    reg JTAG_SEL4_GLBL = 0;

    reg JTAG_USER_TDO1_GLBL = 1'bz;
    reg JTAG_USER_TDO2_GLBL = 1'bz;
    reg JTAG_USER_TDO3_GLBL = 1'bz;
    reg JTAG_USER_TDO4_GLBL = 1'bz;

    assign (weak1, weak0) GSR = GSR_int;
    assign (weak1, weak0) GTS = GTS_int;
    assign (weak1, weak0) PRLD = PRLD_int;

    initial begin
	GSR_int = 1'b1;
	PRLD_int = 1'b1;
	#(ROC_WIDTH)
	GSR_int = 1'b0;
	PRLD_int = 1'b0;
    end

    initial begin
	GTS_int = 1'b1;
	#(TOC_WIDTH)
	GTS_int = 1'b0;
    end

endmodule
`endif
