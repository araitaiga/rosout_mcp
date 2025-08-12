from fastapi import FastAPI
import uvicorn

app = FastAPI()
log_provider = None  # 実行時にセット


@app.get("/logs")
def get_logs(level: str = None, limit: int = 50):
    return {"logs": log_provider.get_logs(level, limit)}


def run_server(provider, host="0.0.0.0", port=8000):
    global log_provider
    log_provider = provider
    uvicorn.run(app, host=host, port=port)
