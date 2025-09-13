"""
Common fixtures and helper functions for testing
"""
import os
from pathlib import Path
import sys
import tempfile

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))
from rosout_mcp.db_manager import InMemoryDatabaseManager


@pytest.fixture
def in_memory_db():
    """In-memory database manager fixture"""
    db_manager = InMemoryDatabaseManager()
    yield db_manager
    db_manager.close()


@pytest.fixture
def temp_file_db():
    """Temporary file database manager fixture"""
    with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as temp_file:
        temp_path = temp_file.name

    db_manager = InMemoryDatabaseManager(temp_path)
    yield db_manager
    db_manager.close()

    # Delete file after test
    if os.path.exists(temp_path):
        os.unlink(temp_path)


@pytest.fixture
def sample_log_data():
    """Sample log data for testing"""
    return [
        (1234567890000000000, "test_node_1", 20, "Info message 1"),
        (1234567891000000000, "test_node_2", 30, "Warning message"),
        (1234567892000000000, "test_node_1", 40, "Error message"),
        (1234567893000000000, "test_node_3", 10, "Debug message"),
        (1234567894000000000, "test_node_2", 20, "Info message 2"),
    ]


def insert_sample_data(db_manager, sample_data):
    """Helper function to insert sample data into database"""
    for timestamp, node, level, message in sample_data:
        db_manager.add_log(timestamp, node, level, message)
