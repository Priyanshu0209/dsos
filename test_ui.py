import asyncio
import logging
from ui.textual_gcs import TextualGCSApp

class DummyGCS:
    drones = ["d1", "d2", "d3"]
    selected_drones = []
    class dispatcher:
        swarm_manager = type('obj', (object,), {'current_formation': 'line'})()

app = TextualGCSApp(DummyGCS())
try:
    app.run()
except Exception as e:
    import traceback
    traceback.print_exc()
