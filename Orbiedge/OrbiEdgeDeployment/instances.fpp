module Orbiedge {

  constant QUEUE_SIZE = 20
  constant STACK_SIZE = 64 * 1024

  instance simBridge: Orbiedge.SimBridge base id 0x10010000 \
    queue size QUEUE_SIZE \
    stack size STACK_SIZE \
    priority 45

  instance fdirManager: Orbiedge.FdirManager base id 0x10011000 \
    queue size QUEUE_SIZE \
    stack size STACK_SIZE \
    priority 44

  instance busMonitor: Orbiedge.BusMonitor base id 0x10012000

  instance uplinkFilter: Orbiedge.AgentUplinkFilter base id 0x10013000

  instance rateGroupDriver: Svc.RateGroupDriver base id 0x10020000

  instance rateGroup10Hz: Svc.ActiveRateGroup base id 0x10021000 \
    queue size QUEUE_SIZE \
    stack size STACK_SIZE \
    priority 43

  instance linuxTimer: Svc.LinuxTimer base id 0x10022000

  instance osTime: Svc.OsTime base id 0x10023000

  instance cmdDisp: Svc.CommandDispatcher base id 0x10030000 \
    queue size QUEUE_SIZE \
    stack size STACK_SIZE \
    priority 40

  instance events: Svc.EventManager base id 0x10031000 \
    queue size QUEUE_SIZE \
    stack size STACK_SIZE \
    priority 39

  instance textLogger: Svc.PassiveTextLogger base id 0x10032000

  instance tlmSend: Svc.TlmChan base id 0x10033000 \
    queue size QUEUE_SIZE \
    stack size STACK_SIZE \
    priority 38

  instance fatalHandler: Svc.FatalHandler base id 0x10034000

  instance fatalAdapter: Svc.AssertFatalAdapter base id 0x10035000

  instance cmdSeq: Svc.CmdSequencer base id 0x10036000 \
    queue size QUEUE_SIZE \
    stack size STACK_SIZE \
    priority 30

}
