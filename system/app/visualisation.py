import asyncio
import json
import websockets


class Visualisation:
    """
    WebSocket server for streaming tracking data to 3D visualization.
    Integrates directly into the System class.
    """
    
    def __init__(self, system, port: int = 8765):
        """
        Parameters
        ----------
        system : System
            Reference to the main System instance
        port : int
            WebSocket server port (default: 8765)
        """
        self.system = system
        self.port = port
        self.clients = set()
        self._server = None
        
    async def register(self, websocket):
        """Register a new client connection."""
        self.clients.add(websocket)
        print(f"[Visualisation] Client connected. Total: {len(self.clients)}")
        
    async def unregister(self, websocket):
        """Unregister a disconnected client."""
        self.clients.remove(websocket)
        print(f"[Visualisation] Client disconnected. Total: {len(self.clients)}")
        
    async def broadcast(self, period=0.1):
        """Continuously broadcast tracking data to all connected clients."""
        while True:
            if self.clients:
                # Get tracked objects and traffic signs from system
                tracks = self.system.tracker.get_all_tracks()
                signs = self.system.traffic_signs
                
                # Prepare data payload
                data = {
                    'tracks': tracks,
                    'signs': signs,
                    'timestamp': asyncio.get_event_loop().time()
                }
                
                message = json.dumps(data)
                
                # Send to all clients
                disconnected = set()
                for client in self.clients:
                    try:
                        await client.send(message)
                    except websockets.exceptions.ConnectionClosed:
                        disconnected.add(client)
                
                # Clean up disconnected clients
                for client in disconnected:
                    await self.unregister(client)
            
            await asyncio.sleep(period)
    
    async def handler(self, websocket):
        """Handle individual WebSocket connections."""
        await self.register(websocket)
        try:
            # Keep connection alive and listen for messages
            async for message in websocket:
                # Can handle client commands here if needed
                pass
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister(websocket)
    
    async def start(self, period=0.1):
        """Start the WebSocket server and broadcasting."""
        print(f"[Visualisation] Starting server on ws://localhost:{self.port}")
        print(f"[Visualisation] Open app/index.html in browser to view")
        
        # Start WebSocket server
        self._server = await websockets.serve(self.handler, "localhost", self.port)
        
        # Start broadcasting task
        await self.broadcast(period)
