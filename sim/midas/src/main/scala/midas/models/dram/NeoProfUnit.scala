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
    neoprofAddr: Long = 0x280000030L, // 0000-1000 reserved for neoprof
    datainWidth: Int = 35
)

class NeoProfiler(params: NeoProfParams)(implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val datain = Input(UInt(params.datainWidth.W)) //TODO: Configurable width
    val recordEn = Input(Bool())
    val readio = Flipped( new NastiReadIO)
  })
  val rden   = Wire(Bool())
  rden := io.readio.ar.valid && !RegNext(io.readio.ar.valid)
  val doread = RegInit(false.B)
  when(io.readio.ar.valid){
    doread := true.B
  }
  when(io.readio.r.valid){
    doread := false.B
  }
  val tmpDataReg = RegInit(0xabc.U(params.datainWidth.W)) // Store the latest accessed page address
  val cmsketchProf = Module(new CMsketch(100,5)(p))
  cmsketchProf.io.datain := io.datain
  cmsketchProf.io.wren   := io.recordEn
  val cmsketch_readio = Wire(new NastiReadIO)
  // cmsketch_readio.ar.valid := (io.readio.ar.valid) && !RegNext(io.readio.ar.valid)
  cmsketch_readio.ar.valid := (io.readio.ar.valid) && !doread
  cmsketch_readio.ar.bits  := io.readio.ar.bits
  cmsketch_readio.r.ready  := io.readio.r.ready
  cmsketchProf.io.readio <> cmsketch_readio

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

  busy := rden || doread 
  chisel3.dontTouch(busy)
  io.readio.ar.ready:= !busy  
  (io.readio.r.valid) := cmsketch_readio.r.valid
  (io.readio.r.bits.data) := cmsketch_readio.r.bits.data
  io.readio.r.bits.resp  := DontCare
  io.readio.r.bits.last  := io.readio.r.valid 
  (io.readio.r.bits.id)  := id
  (io.readio.r.bits.user):= user          
}