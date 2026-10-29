from http.server import HTTPServer

from api.handlers import Handler
import GlobalVariables as state


if __name__ == "__main__":
    server = HTTPServer((state.SERVER_HOST, state.SERVER_PORT), Handler)
    server.serve_forever()
