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
    def test_search_all_data(self, in_memory_db, sample_log_data):
        """Test search without conditions (get all data)"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search()
        assert len(results) == len(sample_log_data)

    @pytest.mark.unit
    def test_search_by_node(self, in_memory_db, sample_log_data):
        """Test search by node name"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        results = query.search(node="test_node_1")
        assert len(results) == 2  # 2 logs for test_node_1

    @pytest.mark.unit
    def test_get_node_list(self, in_memory_db, sample_log_data):
        """Test getting unique node names"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        nodes = query.get_node_list()
        assert "test_node_1" in nodes
        assert "test_node_2" in nodes

    @pytest.mark.unit
    def test_get_database_status(self, in_memory_db, sample_log_data):
        """Test getting database status"""
        insert_sample_data(in_memory_db, sample_log_data)
        query = SQLiteQuery(in_memory_db)

        status = query.get_database_status()
        assert status.record_count == len(sample_log_data)
        assert status.min_timestamp is not None
        assert status.max_timestamp is not None
