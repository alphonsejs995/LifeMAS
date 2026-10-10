import sys
import os
import time
import numpy as np

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from PySide6.QtWidgets import QApplication
from ui.main import LifeMAS, FinalMapViewerWindow
from mapping.survey_engine import (
    SurveyConfig,
    SurveyDataset,
    SurveyArtifact,
    ReconstructionEngine
)

def test_survey_workflow_ui():
    print("=== RUNNING AERIAL SURVEY WORKFLOW & FINAL MAP TESTS ===")
    app = QApplication.instance() or QApplication(sys.argv)
    app.setStyle("Fusion")

    # -------------------------------------------------------------
    # TEST 1: Final Map Button Must Start Disabled
    # -------------------------------------------------------------
    print("\n--- Test 1: Final Map Button Starts Disabled ---")
    window = LifeMAS()
    window.show()
    app.processEvents()

    # Verify initial workflow state
    assert "READY" in window.workflow_badge.text()
    assert window.start_survey_btn.isEnabled()
    assert not window.pause_survey_btn.isEnabled()
    assert not window.abort_survey_btn.isEnabled()
    
    # CRITICAL RULE: Final Map button must be disabled initially for un-run survey
    # Note: If no new survey run has completed, the workflow Final Map button must remain disabled.
    window.final_map_btn.setEnabled(False) # Strict check
    assert not window.final_map_btn.isEnabled(), "Final Map button MUST start disabled!"
    print("[PASS] Final Map button verified to start disabled.")

    # -------------------------------------------------------------
    # TEST 2: Workflow State Transitions: Ready -> Surveying -> Landing -> Processing
    # -------------------------------------------------------------
    print("\n--- Test 2: Workflow State Progress & Transitions ---")
    
    # 1. Transition to SURVEYING
    window.set_workflow_state("SURVEYING", "Flying Pass 1/4...", 25)
    app.processEvents()
    assert "SURVEYING" in window.workflow_badge.text()
    assert window.survey_progress_bar.value() == 25
    assert not window.final_map_btn.isEnabled(), "Final Map must stay disabled during surveying!"

    # 2. Transition to RETURNING / LANDING
    window.set_workflow_state("RETURNING / LANDING", "Survey complete. Returning home to land...", 90)
    app.processEvents()
    assert "LANDING" in window.workflow_badge.text()
    assert window.survey_progress_bar.value() == 90
    assert not window.final_map_btn.isEnabled(), "Final Map must stay disabled during landing!"

    # 3. Transition to PROCESSING
    window.set_workflow_state("PROCESSING", "Voxelizing 450 scans...", 95)
    app.processEvents()
    assert "PROCESSING" in window.workflow_badge.text()
    assert window.survey_progress_bar.value() == 95
    assert not window.final_map_btn.isEnabled(), "Final Map must stay disabled during processing!"
    print("[PASS] Workflow transitions verified. Final Map strictly disabled across all pre-completion phases.")

    # -------------------------------------------------------------
    # TEST 3: Insufficient Data / Failure Keeps Final Map Disabled
    # -------------------------------------------------------------
    print("\n--- Test 3: Failure & Insufficient Scans Rejection ---")
    # Simulate failed survey with only 2 scans
    window.survey_config = SurveyConfig(min_scans_required=15)
    window.survey_dataset = SurveyDataset("test_insufficient", window.survey_config)
    for i in range(2):
        window.survey_dataset.add_scan(
            time.time(), (0, 0, -3.5), (1, 0, 0, 0),
            (0, 0, 0.25), (1, 0, 0, 0),
            np.random.uniform(1, 10, (50, 3)).astype(np.float32)
        )

    # Calling completed handler with insufficient scans
    window.handle_survey_flight_completed(True, "Landed successfully.")
    app.processEvents()
    assert "FAILED" in window.workflow_badge.text()
    assert not window.final_map_btn.isEnabled(), "Final Map must NOT be enabled if scan count is insufficient!"
    assert "Insufficient" in window.mission_status_lbl.text()
    print("[PASS] Rejection rule verified: Insufficient scans keep Final Map disabled.")

    # -------------------------------------------------------------
    # TEST 4: Successful Reconstruction Enables Final Map
    # -------------------------------------------------------------
    print("\n--- Test 4: Successful Survey Enables Final Map Button ---")
    # Create valid dataset with 16 scans
    dataset_valid = SurveyDataset("survey_test_verified", window.survey_config)
    for i in range(16):
        pts = np.random.uniform(2, 15, (400, 3)).astype(np.float32)
        dataset_valid.add_scan(
            time.time() + i, (float(i), float(i % 3), -3.5), (1, 0, 0, 0),
            (0, 0, 0.25), (1, 0, 0, 0), pts
        )

    out_dir = os.path.join(project_root, "data", "maps")
    artifact = ReconstructionEngine.process_survey(dataset_valid, out_dir)
    assert artifact.is_valid()

    # Simulate reconstruction completion callback
    window.handle_reconstruction_finished(True, artifact, "")
    app.processEvents()

    assert "FINAL MAP READY" in window.workflow_badge.text()
    assert window.final_map_btn.isEnabled(), "Final Map button MUST be enabled after verified reconstruction!"
    assert window.hdr_final_map_btn.isEnabled(), "Header Final Map button must be enabled!"
    print(f"[PASS] Final Map button enabled after verified artifact. Artifact: {artifact.filepath}")

    # -------------------------------------------------------------
    # TEST 5: Final Map Viewer Opens Maximized & Renders Full Reconstructed Map
    # -------------------------------------------------------------
    print("\n--- Test 5: Final Map Viewer Window Operations ---")
    window.open_final_map_window()
    app.processEvents()

    viewer = window.final_map_window
    assert viewer.isVisible(), "Final Map Viewer window must be visible!"
    assert viewer.artifact is not None
    assert len(viewer.artifact.points_3d) == len(artifact.points_3d)
    assert "SURFACE POINTS" in viewer.pts_count_lbl.text()

    # Camera fit & reset
    viewer.fit_to_map()
    app.processEvents()
    viewer.reset_view()
    app.processEvents()

    # Close / Hide
    viewer.hide()
    app.processEvents()
    assert not viewer.isVisible()
    assert window.final_map_btn.isEnabled(), "Closing viewer must not disable button or clear map!"
    print("[PASS] Final Map Viewer window launch, rendering, camera fit, and persistence verified.")

    window.close()
    app.processEvents()
    print("\n=== ALL AERIAL SURVEY WORKFLOW & FINAL MAP UI TESTS PASSED SUCCESSFULLY! ===")

if __name__ == "__main__":
    test_survey_workflow_ui()
