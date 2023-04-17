package midas
package models

import freechips.rocketchip.config.Parameters
import junctions._

import chisel3._
import chisel3.util._

import midas.widgets._

import Console.{UNDERLINED, RESET}

// Automatically bound to simulation-memory-mapped. registers Extends this
// bundle to add additional programmable values and instrumentation




abstract class TimingModelWithCMsketchIO(implicit p: Parameters) extends Bundle {
  val tNasti = Flipped(new NastiIO)
  val egressReq = new EgressReq
  val egressResp = Flipped(new EgressResp)
  // This sub-bundle contains all the programmable fields of the model
  val mmReg: MMRegIO
  val docmread = Input(Bool())
}

abstract class TimingModelWithCMsketch(val cfg: BaseConfig)(implicit val p: Parameters) extends Module
    with IngressModuleParameters with EgressUnitParameters with HasNastiParameters {

  // Concrete timing models must implement io with the MMRegIO sub-bundle
  // containing all of the requisite runtime-settings and instrumentation brought
  // out as inputs and outputs respectively. See MMRegIO above.
  // if(true){
  val io: TimingModelWithCMsketchIO
  val longName: String
  // Implemented by concrete timing models to describe their configuration during
  // chisel elaboration
  protected def printTimingModelGenerationConfig: Unit

  def printGenerationConfig: Unit = {
    println("  Timing Model Class: " + longName)
    printTimingModelGenerationConfig
  }

  /**************************** CHISEL BEGINS *********************************/
  // Regulates the return of beats to the target memory system
  val tNasti = io.tNasti
  // Request channels presented to DRAM models
  val nastiReqIden = Module(new IdentityModule(new NastiReqChannels))
  val nastiReq = nastiReqIden.io.out
  val wResp = Wire(Decoupled(new WriteResponseMetaData))
  val rResp = Wire(Decoupled(new ReadResponseMetaData))

  val monitor = Module(new MemoryModelMonitor(cfg))
  monitor.axi4 := io.tNasti

  val tCycle = RegInit(0.U(64.W))
  tCycle := tCycle + 1.U
  io.mmReg.targetCycle.foreach({ _ := tCycle })


  val pendingReads = SatUpDownCounter(cfg.maxReads)
  // pendingReads.inc := tNasti.ar.fire & ((tNasti.ar.bits.addr >= (0x080100000L+100).asUInt)|tNasti.ar.bits.addr<=0x080100000L.asUInt)
  // pendingReads.dec := (tNasti.r.fire && tNasti.r.bits.last) & ((tNasti.ar.bits.addr >= (0x080100000L+100).asUInt)|tNasti.ar.bits.addr<=0x080100000L.asUInt)
  pendingReads.inc := tNasti.ar.fire 
  pendingReads.dec := (tNasti.r.fire && tNasti.r.bits.last) 


  val pendingAWReq = SatUpDownCounter(cfg.maxWrites)
  pendingAWReq.inc := tNasti.aw.fire
  pendingAWReq.dec := tNasti.b.fire

  val pendingWReq = SatUpDownCounter(cfg.maxWrites)
  pendingWReq.inc := tNasti.w.fire && tNasti.w.bits.last
  pendingWReq.dec := tNasti.b.fire

  assert(!tNasti.ar.valid || (tNasti.ar.bits.burst === NastiConstants.BURST_INCR),
    "Illegal ar request: memory model only supports incrementing bursts")

  assert(!tNasti.aw.valid || (tNasti.aw.bits.burst === NastiConstants.BURST_INCR),
    "Illegal aw request: memory model only supports incrementing bursts")

  // Release; returns responses to target
  val xactionRelease = Module(new AXI4ReleaserWithCMsketch)
  xactionRelease.io.docmread := io.docmread
  tNasti.r <> xactionRelease.io.r
  tNasti.b <> xactionRelease.io.b
  io.egressReq <> xactionRelease.io.egressReq
  xactionRelease.io.egressResp <> io.egressResp

  if (cfg.useLLCModel) {
    // Drop the LLC model inline
    val llc_model = Module(new LLCModel(cfg))
    llc_model.io.settings <> io.mmReg.llc.get
    llc_model.io.memRResp <> rResp
    llc_model.io.memWResp <> wResp
    llc_model.io.req.fromNasti(io.tNasti)
    nastiReqIden.io.in <> llc_model.io.memReq
    xactionRelease.io.nextWrite <> llc_model.io.wResp
    xactionRelease.io.nextRead <> llc_model.io.rResp
  } else {
    nastiReqIden.io.in.fromNasti(io.tNasti)
    xactionRelease.io.nextWrite <> wResp
    xactionRelease.io.nextRead <> rResp
  }


  if (cfg.params.xactionCounters) {
    val totalReads = RegInit(0.U(32.W))
    val totalWrites = RegInit(0.U(32.W))
    when(pendingReads.inc){ totalReads := totalReads + 1.U }
    when(pendingAWReq.inc){ totalWrites := totalWrites + 1.U}
    io.mmReg.totalReads foreach { _ := totalReads }
    io.mmReg.totalWrites foreach { _ := totalWrites }
  }

  if (cfg.params.beatCounters) {
    val totalReadBeats = RegInit(0.U(32.W))
    val totalWriteBeats = RegInit(0.U(32.W))
    when(tNasti.r.fire){ totalReadBeats := totalReadBeats + 1.U }
    when(tNasti.w.fire){ totalWriteBeats := totalWriteBeats + 1.U }
    io.mmReg.totalReadBeats foreach { _ := totalReadBeats}
    io.mmReg.totalWriteBeats foreach { _ := totalWriteBeats }
  }

  cfg.params.occupancyHistograms match {
    case Nil => Nil
    case binMaximums =>
      val numBins = binMaximums.size + 1
      val readOutstandingHistogram = Seq.fill(numBins)(RegInit(0.U(32.W)))
      val writeOutstandingHistogram = Seq.fill(numBins)(RegInit(0.U(32.W)))

      def bindHistograms(bins: Seq[UInt], maximums: Seq[Int], count: UInt): Bool = {
        (bins.zip(maximums)).foldLeft(false.B)({ case (hasIncrmented, (bin, maximum)) =>
          when (!hasIncrmented && (count <= maximum.U)) {
            bin :=  bin + 1.U
          }
          hasIncrmented || (count <= maximum.U)
        })
      }

      // Append the largest UInt representable to the end of the Seq to catch remaining cases
      val allBinMaximums = binMaximums :+ -1
      bindHistograms(readOutstandingHistogram, binMaximums, pendingReads.value)
      bindHistograms(writeOutstandingHistogram, binMaximums, pendingAWReq.value)
      io.mmReg.readOutstandingHistogram := readOutstandingHistogram
      io.mmReg.writeOutstandingHistogram := writeOutstandingHistogram
  }
}

// A class of simple timing models that has independently programmable bounds on
// the number of reads and writes the model will accept.
//
// This is in contrast to more complex DRAM models that propogate backpressure
// from shared structures back to the AXI4 request channels.
