"""
Common fixtures for testing
"""
from pathlib import Path
import sys

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
def sample_log_data():
    """Sample log data for testing"""
    return [
        (1234567890000000000, "test_node_1", 20, "Info message 1"),
        (1234567891000000000, "test_node_2", 30, "Warning message"),
        (1234567892000000000, "test_node_1", 40, "Error message"),
    ]


def insert_sample_data(db_manager, sample_data):
    """Helper function to insert sample data into database"""
    for timestamp, node, level, message in sample_data:
        db_manager.add_log(timestamp, node, level, message)
