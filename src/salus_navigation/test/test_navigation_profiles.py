import pytest

from salus_navigation.navigation_profiles import (
    COMPONENTS, NavigationProfileTransaction, TransactionState,
)


def test_profile_commits_only_after_every_component() -> None:
    transaction = NavigationProfileTransaction()
    assert not transaction.begin("rural")
    for component in COMPONENTS:
        transaction.confirm(component, True)
    assert transaction.state == TransactionState.SUCCEEDED
    assert transaction.active_profile == "rural"


def test_partial_failure_rolls_back_confirmed_components() -> None:
    transaction = NavigationProfileTransaction()
    transaction.begin("rural")
    transaction.confirm("ground_filter", True)
    transaction.confirm("local_inflation", False, "rejected")
    assert transaction.state == TransactionState.ROLLING_BACK
    assert transaction.pending == ["ground_filter"]
    transaction.confirm_rollback("ground_filter", True)
    assert transaction.state == TransactionState.FAILED
    assert transaction.active_profile == "urban"


def test_profile_rejects_unknown_and_concurrent_transactions() -> None:
    transaction = NavigationProfileTransaction()
    assert transaction.begin("forest")
    assert not transaction.begin("rural")
    assert transaction.begin("urban")
    with pytest.raises(ValueError):
        transaction.confirm("not-a-component", True)
