# Elesim TURN relay

This optional service relays the two simulator WebRTC streams when direct ICE
connectivity is unavailable. WebRTC packets remain DTLS/SRTP protected while
relayed. Coturn never relays ROS 2/DDS discovery, control/RGBD topics, or the
DDS WebRTC-signaling exchange.

The standalone Compose uses Coturn REST HMAC authentication. Generate its local
configuration from the repository root:

```bash
python3 misc/infra/bootstrap_turn.py \
  --turn-public-ip 203.0.113.10 \
  --turn-realm sim.example.com
```

Start the relay:

```bash
docker compose --env-file misc/infra/coturn/.env \
  -f misc/infra/coturn/compose.yaml up -d
```

Open TCP/UDP `3478` and UDP `49160-49200` in the server firewall. In the
setup-managed profile, mount `misc/infra/generated/turn.secret` into Coturn and
the co-located Simulator. Simulator uses it to issue bounded-lifetime ICE
credentials tied to the active UI session; UI never receives the static HMAC
secret. This deliberately makes Simulator part of the managed TURN trust
boundary. An independently operated external TURN service can instead use
pre-provisioned credentials. Generated secrets and `.env` are ignored by Git.

Elesim's managed-Coturn profile requires SROS2 because the issued credentials
and WebRTC signaling travel over DDS. `trusted-network` may use direct ICE or
an external TURN service with separately provisioned credentials.

TURN can make WebRTC media work through NAT only after UI and Simulator have a
working DDS path for session and SDP exchange. Ordinary IPv4 NAT, CGNAT and
symmetric NAT are unsupported for the direct DDS topology; use a routed VPN or
mutually reachable global IPv6. SSH GUI forwarding is unrelated to TURN and DDS.
