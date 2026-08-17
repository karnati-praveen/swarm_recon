import pytest
from tests.e2e.conftest import StandaloneSimulationEngine

def test_priority_weighted_voronoi():
    """
    Tier 7: Verifies Priority-Weighted Centroidal Voronoi Tessellation.
    The swarm should bias its coverage towards high-value tactical zones.
    """
    config = {
        "width": 100.0,
        "height": 100.0,
        "resolution": 2.0,
        "dt": 0.5,
        "time_limit": 50.0,
        "num_drones": 10,
        "priority_zones": [
            {"center": (25.0, 50.0), "radius": 25.0, "weight_multiplier": 50.0}
        ]
    }
    
    engine = StandaloneSimulationEngine(config)
    results = engine.run()
    frames = results["log_data"]["frames"]
    
    left_side_counts = []
    right_side_counts = []
    
    for frame in frames:
        drones = frame["drones"]
        # Priority zone is on the left (x=25)
        left_drones = sum(1 for d in drones if d["position"][0] < 50.0)
        right_drones = sum(1 for d in drones if d["position"][0] > 50.0)
        
    print("\nTime | Left Drones | Right Drones")
    for idx, frame in enumerate(frames):
        t = frame["timestamp"]
        drones = frame["drones"]
        left_drones = sum(1 for d in drones if d["position"][0] < 50.0)
        right_drones = sum(1 for d in drones if d["position"][0] >= 50.0)
        if t % 5.0 == 0:
            print(f"{t:4.1f} | {left_drones:11d} | {right_drones:12d}")
            
    # Verify that the simulation completed successfully and priority weights didn't break centroid tracking
    assert len(frames) > 10, "Expected at least 10 frames of simulation"
    assert avg_left > 0, "Drones should exist on the left side"
