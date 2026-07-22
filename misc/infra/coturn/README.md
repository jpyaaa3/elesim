# Elesim TURN relay

This optional service relays the two simulator WebRTC streams when direct ICE
connectivity is unavailable. It uses Coturn's REST HMAC authentication; the
router and Coturn must use the same static secret.

Generate local credentials from the repository root:

```bash
python3 misc/infra/bootstrap_security.py \
  --turn-public-ip 203.0.113.10 \
  --turn-realm sim.example.com
```

Start the relay:

```bash
docker compose --env-file misc/infra/coturn/.env \
  -f misc/infra/coturn/compose.yaml up -d
```

Open TCP/UDP `3478` and UDP `49160-49200` in the server firewall. Configure
the router's `turn.static_auth_secret_file` with
`misc/infra/generated/turn.secret`, and replace the public host in its TURN
URL. Generated credentials and `.env` are ignored by Git.
