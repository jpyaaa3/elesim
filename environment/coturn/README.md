# EleSim TURN relay

This service relays the two Sim WebRTC streams when direct ICE connectivity is
unavailable. The normal SROS2 Sim installation includes Coturn in its generated
Compose; `trusted-network` Sim uses direct ICE and does not start a relay. This
standalone helper is retained for manual or legacy deployments. WebRTC packets
remain DTLS/SRTP protected while relayed. Coturn never relays ROS 2/DDS
discovery, control/RGBD topics, or the DDS WebRTC-signaling exchange.

The standalone Compose uses Coturn REST HMAC authentication. Generate its local
configuration from the repository root:

```bash
python3 environment/coturn/generate_credentials.py \
  --turn-public-ip 203.0.113.10 \
  --turn-realm sim.example.com
```

Start the relay:

```bash
docker compose --env-file "$HOME/.local/share/elesim/coturn/.env" \
  -f environment/coturn/compose.yaml up -d
```

Open TCP/UDP `3478` and UDP `49160-49200` in the server firewall. In the
setup-managed profile, the installer keeps the TURN secret under the selected
installation prefix and mounts it only into Coturn and the co-located Sim. Sim
uses it to issue bounded-lifetime ICE credentials tied to the active UI session;
UI never receives the static HMAC secret. This deliberately makes Sim part of
the managed TURN trust boundary. An independently operated external TURN
service can instead use pre-provisioned credentials. The standalone helper
writes its secret and `.env` below `$HOME/.local/share/elesim/coturn` (or
`ELESIM_COTURN_STATE`) rather than into the source checkout.

EleSim's managed-Coturn profile requires SROS2 because the issued credentials
and WebRTC signaling travel over DDS. `trusted-network` uses direct ICE only;
legacy external TURN states may use separately provisioned credentials.

TURN can make WebRTC media work through NAT only after UI and Sim have a
working DDS path for session and SDP exchange. Ordinary IPv4 NAT, CGNAT and
symmetric NAT are unsupported for the direct DDS topology; use a routed VPN or
mutually reachable global IPv6. SSH GUI forwarding is unrelated to TURN and DDS.
