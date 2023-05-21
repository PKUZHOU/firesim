package midas.models

import chisel3._
import chisel3.util._
import junctions._
import freechips.rocketchip.util.{DecoupledHelper, ParameterizedBundle, HellaPeekingArbiter}
import freechips.rocketchip.config.{Parameters, Field}
import freechips.rocketchip.amba.axi4._

case class NeoProfParams(
    CMsketchWidth: Int = 32,
    CMsketchDepth: Int = 4,
    neoprofAddr: Long = 0x280100030L,
    datainWidth: Int = 35
)

class NeoProfiler(params: NeoProfParams)(implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val datain = Input(UInt(params.datainWidth.W)) //TODO: Configurable width
    val recordEn = Input(Bool())
    val readio = Flipped( new NastiReadIO)
  })

  val tmpDataReg = RegInit(0xabc.U(params.datainWidth.W)) // Store the latest accessed page address

  when(io.recordEn){
    tmpDataReg := io.datain
  }

  val busy = RegInit(false.B)
  val id = RegInit(0.U(4.W))
  val user = RegInit(0.U(1.W))
  when(io.readio.ar.valid){
    id := io.readio.ar.bits.id
    user := io.readio.ar.bits.user
  }
  when(io.readio.ar.ready && !io.readio.ar.valid){
    id := 0.U
    user := 0.U
  }
  val rden   = Wire(Bool())
  rden := io.readio.ar.valid && !RegNext(io.readio.ar.valid)
  val doread = RegInit(false.B)
  when(io.readio.ar.valid){
    doread := true.B
  }
  when(io.readio.r.valid){
    doread := false.B
  }
  busy := rden || doread 
  chisel3.dontTouch(busy)
  io.readio.ar.ready:= !busy  
  (io.readio.r.valid) := doread
  (io.readio.r.bits.data) := tmpDataReg
  io.readio.r.bits.resp  := DontCare
  io.readio.r.bits.last  := io.readio.r.valid 
  (io.readio.r.bits.id)  := id
  (io.readio.r.bits.user):= user          
}