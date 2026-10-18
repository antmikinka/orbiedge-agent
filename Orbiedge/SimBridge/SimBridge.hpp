#ifndef ORBIEDGE_SIMBRIDGE_HPP
#define ORBIEDGE_SIMBRIDGE_HPP

#include "Orbiedge/SimBridge/SimBridgeComponentAc.hpp"

#include "Os/Mutex.hpp"

namespace Orbiedge {

class SimBridge final : public SimBridgeComponentBase {
  public:
    explicit SimBridge(const char* const compName);
    ~SimBridge() override;

    void startSockets();

  private:
    static constexpr U32 LINE_MAX = 1024;
    static constexpr U32 MODE_CLIENTS = 4;
    static constexpr U32 UPLINK_CLIENTS = 2;
    static constexpr int SENSOR_PORT = 5600;
    static constexpr int MODE_PORT = 5601;
    static constexpr int UPLINK_PORT = 50001;
    static constexpr U64 SENSOR_LOSS_MS = 500ULL;

    struct Client {
        int fd;
        char line[LINE_MAX];
        U32 length;
    };

    struct ParsedSensor {
        U64 simTimeNs;
        F64 soc;
        F64 solar;
        F64 wheels[4];
        F64 tempC;
        bool heaterOn;
    };
    struct ParsedProposal {
        U32 opcode;
        Mode action;
        U8 wheelIndex;
        F32 confidence;
        U64 simTimeNs;
    };

    void run_handler(FwIndexType portNum, U32 context) override;
    void modeIn_handler(FwIndexType portNum, U64 simTimeNs, const Mode& mode, U8 wheelIndex) override;

    void bindListener(int port, int& fd);
    void acceptClients(int listenFd, Client* clients, U32 capacity, bool uplink);
    void readUplink(Client& client, ParsedProposal* out, U32 capacity, U32& count);
    void readSensor(ParsedSensor* out, U32 capacity, U32& count);
    void connectSensor();
    void writeMode(U64 simTimeNs, Mode mode, U8 wheelIndex);
    void closeFd(int& fd);
    void rememberMode(U64 simTimeNs, Mode mode, U8 wheelIndex);
    static U64 wallMs();
    static bool parseSensor(const char* line, U64& simTimeNs, F64& soc, F64& solar, F64 wheels[4], F64& tempC,
                            bool& heaterOn);
    static bool parseProposal(const char* line, U32& opcode, Mode& action, U8& wheelIndex, F32& confidence,
                              U64& simTimeNs);

    Os::Mutex m_mutex;
    bool m_started;
    int m_sensorFd;
    int m_sensorState;
    int m_modeListen;
    int m_uplinkListen;
    Client m_modeClients[MODE_CLIENTS];
    Client m_uplinkClients[UPLINK_CLIENTS];
    char m_sensorLine[LINE_MAX];
    U32 m_sensorLength;
    bool m_haveSample;
    U64 m_lastSampleMs;
    bool m_sensorLost;
    bool m_haveMode;
    U64 m_modeTime;
    Mode m_mode;
    U8 m_wheel;
};

}  // namespace Orbiedge

#endif
