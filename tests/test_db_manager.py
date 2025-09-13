"""
Unit tests for db_manager.py
"""
from conftest import insert_sample_data
import pytest

from rosout_mcp.db_manager import DatabaseManager
from rosout_mcp.db_manager import InMemoryDatabaseManager


class TestDatabaseManager:
    """Test class for DatabaseManager"""

    @pytest.mark.unit
    def test_init_in_memory(self):
        """Test in-memory database initialization"""
        db_manager = DatabaseManager()
        assert db_manager.db_path == ":memory:"
        db_manager.close()

    @pytest.mark.unit
    def test_connection_creation(self, in_memory_db):
        """Test database connection creation"""
        connection = in_memory_db.connection
        assert connection is not None

    @pytest.mark.unit
    def test_add_log(self, in_memory_db):
        """Test adding log entries"""
        in_memory_db.add_log(1234567890, "test_node", 20, "test message")

        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == 1

    @pytest.mark.unit
    def test_clear_logs(self, in_memory_db, sample_log_data):
        """Test clearing logs"""
        insert_sample_data(in_memory_db, sample_log_data)

        # Verify data exists
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == len(sample_log_data)

        # Clear and verify
        in_memory_db.clear_logs()
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == 0

    @pytest.mark.unit
    def test_execute_query(self, in_memory_db, sample_log_data):
        """Test executing queries"""
        insert_sample_data(in_memory_db, sample_log_data)

        result = in_memory_db.execute(
            "SELECT * FROM logs WHERE node = ?", ("test_node_1",))
        assert len(result) == 2


class TestInMemoryDatabaseManager:
    """Test class for InMemoryDatabaseManager"""

    @pytest.mark.unit
    def test_init(self):
        """Test initialization"""
        db_manager = InMemoryDatabaseManager()
        assert db_manager.db_path == ":memory:"
        db_manager.close()
