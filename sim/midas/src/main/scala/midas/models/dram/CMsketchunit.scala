package midas.models

import chisel3._
import chisel3.util._
import junctions._
import freechips.rocketchip.util.{DecoupledHelper, ParameterizedBundle, HellaPeekingArbiter}
import freechips.rocketchip.config.{Parameters, Field}
import freechips.rocketchip.amba.axi4._
import scala.math.pow



class CMsketch(w:Int , d:Int)(implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val datain = Input(UInt(35.W))
    val wren = Input(Bool())
    val readio =Flipped( new NastiReadIO)
  })

  def Hash(x: UInt, n: UInt, m: Int): UInt = {
    val k = Wire(UInt(35.W))
    k := (x << m.U)
    (k%n)
  }

  def countgen(max: UInt, en: Bool): UInt = {
    val cnt = RegInit(0.U(35.W))
    when(en) {
      cnt := Mux(cnt === max-1.U, 0.U, cnt + 1.U)
    }
    cnt
  }

  val Hashin = Wire(Vec(d, UInt(log2Up(w).W)))
  val Hashrd = Wire(Vec(d, UInt(log2Up(w).W)))
  val counter =Wire(Vec(w * d, UInt(35.W)))
  val enable = Wire(Vec(w * d, Bool()))

  for (i <- 0 until d) {
    when(io.wren) {
      Hashin(i) := Hash(io.datain, w.U, i + 1)
    }.otherwise{
      Hashin(i) := w.U
    }
  }

  when(io.wren){
  for (i <- 0 until d) {
    for (j <- 0 until w) {
      enable(i * w + j) := j.U === Hashin(i)
    }
  }}.otherwise{
    for (i <- 0 until d) {
      for (j <- 0 until w) {
        enable(i * w + j) := false.B
      }
    }
  }

    for (i <- 0 until d) {
      for (j <- 0 until w) {
        counter(i*w+j) := countgen(10000000.U, enable(i*w+j))
      }
    }

  val rden   = Wire(Bool())
  rden := io.readio.ar.valid & !RegNext(io.readio.ar.valid)

  val doread = RegInit(false.B)
  when(io.readio.ar.valid){
    doread := true.B
  }
  when(io.readio.r.valid){
    doread := false.B
  }

  val dataoutpre = Wire(Vec(d,UInt(35.W)))
  val x = Wire(Bool())
  x := io.readio.ar.valid | doread

  val datard = RegInit(0.U(35.W))
  when(io.readio.ar.valid){
    datard := io.readio.ar.bits.addr
  }
  when(io.readio.r.valid){
    datard := 0.U(35.W)
  }
  for(i <- 0 until(d)){
    Hashrd(i) := Mux(x,Hash(datard,w.U,i+1),0.U)
  }
  val findmin = Wire(Bool())
  findmin := rden & !RegNext(rden)
  when(findmin){
    for(i <- 0 until d){
      dataoutpre(i) := counter(i.U*w.U + Hashrd(i))
    }
  }.otherwise{
    for (i <- 0 until d) {
      dataoutpre(i) := 0.U
    }
  }

  val datamin = RegInit(0.U(35.W))

  when(findmin){
    for(i<-0 until(d)){
      when (i.U===0.U){
        datamin := dataoutpre(0)
      }.otherwise{
        when(dataoutpre(i)<datamin){
          datamin := dataoutpre(i)
        }
      }
    }
  }.otherwise{
    datamin := 10000000.U
  }

  val temp = RegInit(0.U(35.W))
  val tempwren = RegInit(false.B)

  when(io.wren){
    temp := io.datain
    tempwren := true.B
  }.otherwise{
    when((io.readio.r.valid)){
      temp := 0.U
      tempwren := false.B
    }
  }

  val busy = RegInit(false.B)
  val id = RegInit(0.U(4.W))
  val user = RegInit(0.U(1.W))
  when(io.readio.ar.valid){
    id := io.readio.ar.bits.id
    user := io.readio.ar.bits.user
  }
  when(io.readio.ar.ready & !io.readio.ar.valid){
    id := 0.U
    user := 0.U
  }
  busy := rden | doread 
  chisel3.dontTouch(busy)
  io.readio.ar.ready:= !busy  
  (io.readio.r.valid) := doread & !(datamin === 10000000.U)
  (io.readio.r.bits.data) := Mux(io.readio.r.valid,Mux(tempwren & (temp === io.readio.ar.bits.addr),datamin + 1.U,datamin),0.U)
  io.readio.r.bits.resp  := DontCare
  io.readio.r.bits.last  := io.readio.r.valid 
  (io.readio.r.bits.id)  := id
  (io.readio.r.bits.user):= user          
}
