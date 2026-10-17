module Orbiedge {

  @ Passive filter in front of CmdDispatcher source port 1.
  @ Forwards only AGENT.PROPOSE that passes the stale-time and wall-rate checks.
  @ FDIR.CLEAR and every other opcode increment CommandsDropped.
  passive component AgentUplinkFilter {

    sync input port uplinkIn: Proposal

    output port forwardOut: Proposal

    telemetry CommandsDropped: U32 id 0

    import Fw.Channel

    time get port timeGetOut

  }

}
