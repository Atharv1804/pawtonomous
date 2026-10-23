# Hermes Agent settings used by Jarvis

Hermes Agent v0.22.4 runs on the Pi. Secrets (OpenRouter / Gemini / Telegram / API-server keys) live only in
`~/.hermes/.env` and are **not** in this repo. These commands reproduce the rest of the setup.

```bash
H=~/.local/bin/hermes

# Main model: gpt-oss-20b in LM Studio on the Mac, reached through the SSH tunnel (Pi 127.0.0.1:1234)
$H config set model.provider lmstudio
$H config set model.base_url http://127.0.0.1:1234/v1
$H config set model.default gpt-oss-20b
$H config set model.context_length 65536 --force       # Hermes needs >= 64k for reliable tool use
$H config set fallback_model '[{"provider": "gemini", "model": "gemini-3.1-flash-lite"}, {"provider": "openrouter", "model": "nvidia/nemotron-3.5-lightning:free"}]'

# Speed: these cut a robot question from ~15 s to ~6 s
$H config set agent.reasoning_effort low --force
$H config set agent.api_max_retries 2
$H config set tools.tool_search.enabled off --force                       # no tool search round trips
$H config set auxiliary.title_generation.model_upgrade_enabled false --force
$H config set auxiliary.background_review.enabled false --force

# Robot tools (MCP) and photo delivery to Telegram
$H config set mcp_servers '{"robot_car": {"command": "/home/<you>/robot_ws/tools/.venv/bin/python", "args": ["/home/<you>/robot_ws/tools/robot_mcp_server.py"], "env": {"ROBOT_API_URL": "http://127.0.0.1:8765", "ROBOT_AGENT": "hermes"}}}'
$H config set gateway.media_delivery_allow_dirs '["/home/<you>/robot_ws/captures"]'

# Fewer tools = smaller prompt = faster answers
$H tools disable --platform api_server web browser terminal file code_execution image_gen skills todo session_search connections delegation
$H tools disable --platform telegram browser computer_use image_gen delegation code_execution web skills connections session_search todo

# Voice on the Pi itself
$H pm install --extra messaging --extra voice --extra piper
$H config set tts.provider piper
```

`~/.hermes/.env` needs (values not shown): `OPENROUTER_API_KEY`, `GEMINI_API_KEY`, `TELEGRAM_BOT_TOKEN`,
`TELEGRAM_ALLOWED_USERS` (your Telegram user id only), `TELEGRAM_PROXY=socks5://127.0.0.1:1080`,
`HTTPS_PROXY=socks5h://127.0.0.1:1080` + `NO_PROXY=...` (Google via the Mac in China),
`API_SERVER_ENABLED=true`, `API_SERVER_HOST=127.0.0.1`, `API_SERVER_PORT=8642`, `API_SERVER_KEY`,
`HERMES_STREAM_STALE_TIMEOUT=20`, `HF_ENDPOINT=https://hf-mirror.com`, `HF_HUB_DISABLE_XET=1`.

Copy `SOUL.md` (Jarvis's personality, robot brief and command-to-tool table) to `~/.hermes/SOUL.md`.
