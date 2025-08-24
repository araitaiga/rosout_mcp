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
    def test_search_by_time_range_both_none(self, in_memory_db, sample_log_data):
        """Time range search: both None (get all data)"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_time_range(None, None)
        assert len(results) == len(sample_log_data)

    @pytest.mark.unit
    def test_search_by_time_range_start_only(self, in_memory_db, sample_log_data):
        """Time range search: start time only specified"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        # Specify middle timestamp
        start_time = 1234567891000000000
        results = query.search_by_time_range(start_time, None)
        assert len(results) == 4  # Data from start_time onwards

    @pytest.mark.unit
    def test_search_by_time_range_end_only(self, in_memory_db, sample_log_data):
        """Time range search: end time only specified"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        # Specify middle timestamp
        end_time = 1234567892000000000
        results = query.search_by_time_range(None, end_time)
        assert len(results) == 3  # Data up to end_time

    @pytest.mark.unit
    def test_search_by_time_range_both_specified(self, in_memory_db, sample_log_data):
        """Time range search: both start and end time specified"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        start_time = 1234567891000000000
        end_time = 1234567893000000000
        results = query.search_by_time_range(start_time, end_time)
        assert len(results) == 3  # Data within specified range

    @pytest.mark.unit
    def test_search_by_node(self, in_memory_db, sample_log_data):
        """Node name search test"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_node("test_node_1")
        assert len(results) == 2  # 2 logs for test_node_1

        # Verify all results are for test_node_1
        for row in results:
            assert row[1] == "test_node_1"

    @pytest.mark.unit
    def test_search_by_node_nonexistent(self, in_memory_db, sample_log_data):
        """Search test for nonexistent node name"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_node("nonexistent_node")
        assert len(results) == 0

    @pytest.mark.unit
    def test_search_by_level_range_both_none(self, in_memory_db, sample_log_data):
        """Level range search: both None (get all data)"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_level_range(None, None)
        assert len(results) == len(sample_log_data)

    @pytest.mark.unit
    def test_search_by_level_range_min_only(self, in_memory_db, sample_log_data):
        """Level range search: minimum level only specified"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_level_range(20, None)  # INFO and above
        assert len(results) == 4  # 4 entries excluding DEBUG (10)

    @pytest.mark.unit
    def test_search_by_level_range_max_only(self, in_memory_db, sample_log_data):
        """Level range search: maximum level only specified"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_level_range(None, 20)  # INFO and below
        assert len(results) == 3  # DEBUG, INFO x2

    @pytest.mark.unit
    def test_search_by_level_range_both_specified(self, in_memory_db, sample_log_data):
        """Level range search: both minimum and maximum level specified"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_level_range(20, 30)  # INFO to WARNING
        assert len(results) == 3  # INFO x2, WARNING x1

    @pytest.mark.unit
    def test_search_by_message(self, in_memory_db, sample_log_data):
        """Message search test"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_message("Info")
        assert len(results) == 2  # "Info message 1" and "Info message 2"

        # Verify all results contain the keyword
        for row in results:
            assert "Info" in row[3]

    @pytest.mark.unit
    def test_search_by_message_case_insensitive(self, in_memory_db, sample_log_data):
        """Case-insensitive message search test"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_message("error")  # Search with lowercase
        assert len(results) == 1  # "Error message" should be found

    @pytest.mark.unit
    def test_search_by_message_nonexistent(self, in_memory_db, sample_log_data):
        """Message search test with nonexistent keyword"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search_by_message("nonexistent")
        assert len(results) == 0

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
