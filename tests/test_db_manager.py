"""
Unit tests for db_manager.py
"""
import os
from pathlib import Path
import tempfile

from conftest import insert_sample_data
import pytest

from rosout_mcp.db_manager import DatabaseManager
from rosout_mcp.db_manager import FileDatabaseManager
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
    def test_init_with_file_path(self):
        """Test database initialization with file path specified"""
        test_path = "/tmp/test.db"
        db_manager = DatabaseManager(test_path)
        assert db_manager.db_path == test_path
        db_manager.close()

    @pytest.mark.unit
    def test_connection_creation(self, in_memory_db):
        """Test database connection creation"""
        connection = in_memory_db.connection
        assert connection is not None
        # Verify that table is created
        cursor = connection.cursor()
        cursor.execute("""
            SELECT name FROM sqlite_master
            WHERE type='table' AND name='logs'
        """)
        assert cursor.fetchone() is not None

    @pytest.mark.unit
    def test_clear_logs_empty_database(self, in_memory_db):
        """Test clear_logs on empty database"""
        # Verify no error occurs on empty database
        in_memory_db.clear_logs()

    @pytest.mark.unit
    def test_clear_logs_with_data(self, in_memory_db, sample_log_data):
        """Test clear_logs with data present"""
        # Insert data
        insert_sample_data(in_memory_db, sample_log_data)

        # Verify data has been inserted
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == len(sample_log_data)

        # Clear logs
        in_memory_db.clear_logs()

        # Verify data has been cleared
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == 0

    @pytest.mark.unit
    def test_add_log_multiple(self, in_memory_db):
        """Test multiple add_log calls"""
        in_memory_db.add_log(1234567890, "test_node_1", 20, "test message 1")
        in_memory_db.add_log(1234567891, "test_node_2", 30, "test message 2")

        # Verify both records were inserted
        result = in_memory_db.execute("SELECT COUNT(*) FROM logs")
        assert result[0][0] == 2

    @pytest.mark.unit
    def test_execute_with_params(self, in_memory_db, sample_log_data):
        """Test execution of parameterized queries"""
        insert_sample_data(in_memory_db, sample_log_data)

        # Execute parameterized query
        result = in_memory_db.execute(
            "SELECT * FROM logs WHERE node = ?",
            ("test_node_1",)
        )
        assert len(result) == 2  # 2 logs for test_node_1

    @pytest.mark.unit
    def test_execute_without_params(self, in_memory_db, sample_log_data):
        """Test execution of queries without parameters"""
        insert_sample_data(in_memory_db, sample_log_data)

        result = in_memory_db.execute("SELECT * FROM logs")
        assert len(result) == len(sample_log_data)

    @pytest.mark.unit
    def test_execute_on_nonexistent_table(self, in_memory_db):
        """Test query on nonexistent table"""
        # Drop table
        in_memory_db.connection.execute("DROP TABLE IF EXISTS logs")

        # Query on nonexistent table should return empty list
        result = in_memory_db.execute("SELECT * FROM logs")
        assert result == []

    @pytest.mark.unit
    def test_context_manager(self):
        """Test context manager"""
        with DatabaseManager() as db_manager:
            assert db_manager.db_path == ":memory:"
            connection = db_manager.connection
            assert connection is not None
        # After exiting with statement, verify close was called
        assert db_manager._connection is None


class TestInMemoryDatabaseManager:
    """Test class for InMemoryDatabaseManager"""

    @pytest.mark.unit
    def test_init(self):
        """Test initialization"""
        db_manager = InMemoryDatabaseManager()
        assert db_manager.db_path == ":memory:"
        db_manager.close()


class TestFileDatabaseManager:
    """Test class for FileDatabaseManager"""

    @pytest.mark.unit
    def test_init_with_valid_path(self):
        """Test initialization with valid path"""
        test_path = "/tmp/test.db"
        db_manager = FileDatabaseManager(test_path)
        assert db_manager.db_path == test_path
        db_manager.close()

    @pytest.mark.unit
    def test_init_with_none_path(self):
        """Test initialization error with None path"""
        with pytest.raises(ValueError, match="File database manager requires a valid db_path"):
            FileDatabaseManager(None)

    @pytest.mark.unit
    def test_file_database_persistence(self):
        """Test file database persistence"""
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as temp_file:
            temp_path = temp_file.name

        try:
            # Insert data with first database connection
            with FileDatabaseManager(temp_path) as db_manager1:
                db_manager1.add_log(
                    1234567890, "test_node", 20, "test message")

            # Verify data persistence with new connection
            with FileDatabaseManager(temp_path) as db_manager2:
                result = db_manager2.execute("SELECT COUNT(*) FROM logs")
                assert result[0][0] == 1

        finally:
            # Clean up file
            if os.path.exists(temp_path):
                os.unlink(temp_path)
