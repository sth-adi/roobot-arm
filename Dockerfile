FROM python:3.12-slim

ENV PYTHONUNBUFFERED=1
WORKDIR /app
COPY server.py .

# Where to forward every received controller state (empty = don't forward)
ENV FORWARD_URL=""
EXPOSE 8000

HEALTHCHECK --interval=10s --timeout=3s --retries=3 \
  CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/health', timeout=2)"

USER nobody
CMD ["python", "server.py", "--host", "0.0.0.0", "--port", "8000"]
