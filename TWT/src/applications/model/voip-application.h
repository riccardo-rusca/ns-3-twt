#ifndef VOIP_APPLICATION_H
#define VOIP_APPLICATION_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
// #include "ns3/data-rate.h"
#include "ns3/random-variable-stream.h"
#include "ns3/traced-callback.h"
#include "ns3/seq-ts-size-header.h"

const uint32_t ACTIVE_STATE = 1;
const uint32_t SILENT_STATE = 2;
const double FRAME_INTERVAL[2] = {0.02, 0.16};

namespace ns3 {

class Address;
class RandomVariableStream;
class Socket;

class VoiPApplication : public Application 
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  VoiPApplication ();

  virtual ~VoiPApplication();

  /**
   * \brief Return a pointer to associated socket.
   * \return pointer to associated socket
   */
  Ptr<Socket> GetSocket (void) const;

protected:
  virtual void DoDispose (void);
private:
  // inherited from Application base class.
  virtual void StartApplication (void);    // Called at time specified by Start
  virtual void StopApplication (void);     // Called at time specified by Stop

  //helpers
  /**
   * \brief Cancel all pending events.
   */
  void CancelEvents ();

  // Event handlers
  /**
   * \brief Send a packet
   */
  void SendPacket ();

  Ptr<Socket>     m_socket;       //!< Associated socket
  Address         m_peer;         //!< Peer address
  Address         m_local;        //!< Local address to bind to
  bool            m_connected;    //!< True if connected
  double          m_expMean;      //!< Rate that data is generated
  double          m_ssprob;
  uint32_t        m_currState;
  uint32_t        m_nextState;
  uint32_t        m_ActivePktSize;      //!< Size of packets in active state
  uint32_t        m_SilentPktSize;      //!< Size of packets in silent state
  uint32_t        m_pktSize;
  uint32_t        m_numTotalPkts;
  uint32_t        m_numRemainingPkts;
  uint64_t        m_totBytes;     //!< Total bytes sent so far
  EventId         m_stateStartEvent;     //!< Event id for next start or stop event
  EventId         m_sendEvent;    //!< Event id of pending "send packet" event
  TypeId          m_tid;          //!< Type of the socket used
  uint32_t        m_seq {0};      //!< Sequence
  Ptr<Packet>     m_unsentPacket; //!< Unsent packet cached for future attempt
  bool            m_enableSeqTsSizeHeader {false}; //!< Enable or disable the use of SeqTsSizeHeader
  Ptr<UniformRandomVariable> randU = CreateObject<UniformRandomVariable> ();


  /// Traced Callback: transmitted packets.
  TracedCallback<Ptr<const Packet> > m_txTrace;

  /// Callbacks for tracing the packet Tx events, includes source and destination addresses
  TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_txTraceWithAddresses;

  /// Callback for tracing the packet Tx events, includes source, destination, the packet sent, and header
  TracedCallback<Ptr<const Packet>, const Address &, const Address &, const SeqTsSizeHeader &> m_txTraceWithSeqTsSize;

private:
  /**
   * \brief Schedule the next packet transmission
   */
  void ScheduleNextTx ();
  /**
   * \brief Schedule the next On period start
   */
  void StateStart ();
  /**
   * \brief Schedule the next Off period start
   */
  void GenerateNextState ();
  /**
   * \brief Handle a Connection Succeed event
   * \param socket the connected socket
   */
  void ConnectionSucceeded (Ptr<Socket> socket);
  /**
   * \brief Handle a Connection Failed event
   * \param socket the not connected socket
   */
  void ConnectionFailed (Ptr<Socket> socket);
};

} // namespace ns3

#endif /* ONOFF_APPLICATION_H */
