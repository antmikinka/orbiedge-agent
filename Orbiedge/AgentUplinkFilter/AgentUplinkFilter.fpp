module Orbiedge {

  @ Passes opcode 0 only. Drops FDIR.CLEAR and every other opcode.
  @ Staleness uses sim time. The one-proposal cap uses wall time.
  passive component AgentUplinkFilter {

    sync input port proposalIn: Proposal

    sync input port setTime: SimTimeStamp

    output port acceptOut: Proposal

    telemetry Drops: U32 id 0

    telemetry Accepts: U32 id 1

    import Fw.Channel

    import Fw.Event

    time get port timeGetOut

  }

}
