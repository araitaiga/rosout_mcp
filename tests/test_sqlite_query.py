"""
Unit tests for sqlite_query.py
"""
from conftest import insert_sample_data
import pytest

from rosout_mcp.sqlite_query import SQLiteQuery


class TestSQLiteQuery:
    """Test class for SQLiteQuery"""

    @pytest.mark.unit
    def test_init_with_valid_db_manager(self, in_memory_db):
        """Test initialization with valid database manager"""
        query = SQLiteQuery(in_memory_db)
        assert query.db_manager == in_memory_db

    @pytest.mark.unit
    def test_init_with_none_db_manager(self):
        """Test initialization error with None database manager"""
        with pytest.raises(ValueError, match="db_manager is required"):
            SQLiteQuery(None)

    @pytest.mark.unit
    def test_search_no_conditions(self, in_memory_db, sample_log_data):
        """No conditions search (get all data) test"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search()
        assert len(results) == len(sample_log_data)

    @pytest.mark.unit
    def test_search_single_condition(self, in_memory_db, sample_log_data):
        """Single condition search test"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        # Specify node name only
        results = query.search(node="test_node_2")
        assert len(results) == 2  # 2 logs for test_node_2

    @pytest.mark.unit
    def test_search_multiple_conditions(self, in_memory_db, sample_log_data):
        """Multiple conditions search test"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        # Combine multiple conditions
        results = query.search(
            start_time=1234567891000000000,
            end_time=1234567894000000000,
            node="test_node_2",
            min_level=20,
            message="Info"
        )
        # Only "Info message 2" matches all conditions
        assert len(results) == 1

    @pytest.mark.unit
    def test_search_complex_time_and_level_filter(self, in_memory_db, sample_log_data):
        """Complex time and level filter test"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        # Logs at error level and above within specific time range
        results = query.search(
            start_time=1234567890000000000,
            end_time=1234567894000000000,
            min_level=40  # ERROR and above
        )
        assert len(results) == 1  # Only Error message

    @pytest.mark.unit
    def test_search_empty_database(self, in_memory_db):
        """Search test on empty database"""
        query = SQLiteQuery(in_memory_db)

        results = query.search()
        assert len(results) == 0

    @pytest.mark.unit
    def test_search_no_matching_results(self, in_memory_db, sample_log_data):
        """Test when no data matches search conditions"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        # Search with nonexistent node
        results = query.search(node="nonexistent_node")
        assert len(results) == 0

    @pytest.mark.unit
    def test_execute_direct_query(self, in_memory_db, sample_log_data):
        """Direct test of _execute method"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        # Execute direct SQL query
        results = query._execute("SELECT COUNT(*) FROM logs")
        assert results[0][0] == len(sample_log_data)

    @pytest.mark.unit
    def test_execute_with_parameters(self, in_memory_db, sample_log_data):
        """Test _execute method with parameters"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        # Execute parameterized query
        results = query._execute(
            "SELECT * FROM logs WHERE level >= ?", (30,)
        )
        assert len(results) == 2  # WARNING and ERROR logs
