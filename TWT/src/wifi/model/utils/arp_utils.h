#include "ns3/core-module.h"
#include "ns3/arp-cache.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/node-list.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-interface.h"

using namespace ns3;

// Function to populate ARP cache of all nodes in ns-3, prevents ARP blocking in Wi-Fi networks
// Execute after assigning IP addresses
// Author: Pavel Boyko
// https://groups.google.com/d/msg/ns-3-users/L_ZfZ9L1_b0/dCuzc859ENcJ

// Taken and slightly adapted from: https://gist.github.com/SzymonSzott/de5c431d687f7b3a0b10743af6ac7ce2
void PopulateARPcache() {
  Ptr<ArpCache> arp = CreateObject<ArpCache> ();
  arp->SetAliveTimeout (Seconds (3600 * 24 * 365) );

  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
  {
    Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
    NS_ASSERT (ip !=nullptr);
    ObjectVectorValue interfaces;
    ip->GetAttribute ("InterfaceList", interfaces);

    for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j++)
    {
      Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
      NS_ASSERT (ipIface != nullptr);
      Ptr<NetDevice> device = ipIface->GetDevice ();
      NS_ASSERT (device != nullptr);
      Mac48Address addr = Mac48Address::ConvertFrom (device->GetAddress () );

      for (uint32_t k = 0; k < ipIface->GetNAddresses (); k++)
      {
              Ipv4Address ipAddr = ipIface->GetAddress (k).GetLocal();
              if (ipAddr == Ipv4Address::GetLoopback ())
                      continue;

              ArpCache::Entry *entry = arp->Add (ipAddr);
              Ipv4Header ipv4Hdr;
              ipv4Hdr.SetDestination (ipAddr);
              Ptr<Packet> p = Create<Packet> (100);
              entry->MarkWaitReply (ArpCache::Ipv4PayloadHeaderPair (p, ipv4Hdr));
              entry->MarkAlive (addr);
      }
    }
  }

  for (NodeList::Iterator i = NodeList::Begin (); i != NodeList::End (); ++i)
  {
    Ptr<Ipv4L3Protocol> ip = (*i)->GetObject<Ipv4L3Protocol> ();
    NS_ASSERT (ip !=nullptr);
    ObjectVectorValue interfaces;
    ip->GetAttribute ("InterfaceList", interfaces);

    for (ObjectVectorValue::Iterator j = interfaces.Begin (); j != interfaces.End (); j ++)
    {
      Ptr<Ipv4Interface> ipIface = (*j).second->GetObject<Ipv4Interface> ();
      ipIface->SetAttribute ("ArpCache", PointerValue (arp) );
    }
  }
}
