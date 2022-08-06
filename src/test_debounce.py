from debounce import Debounce

def test_create_debounce():
    low_value = 5
    high_value = 10

    debounce = Debounce(1.0, 10, low_value, high_value, high_value)
    assert debounce.current_value() == high_value

def test_debounce_to_low_value():
    debounce_time = 1.0
    sample_frequency = 5
    low_value = 5
    high_value = 10

    debounce = Debounce(debounce_time, sample_frequency, low_value, high_value, high_value)

    for x in range(4):
        debounce.record_low_value()
        assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == low_value

def test_debounce_to_high_value():
    debounce_time = 1.0
    sample_frequency = 5
    low_value = 5
    high_value = 10

    debounce = Debounce(debounce_time, sample_frequency, low_value, high_value, low_value)

    for x in range(4):
        debounce.record_high_value()
        assert debounce.current_value() == low_value

    debounce.record_high_value()
    assert debounce.current_value() == high_value

def test_with_jitter_no_value_change():
    # real signal 11111100000
    # corrupted   10110110010
    # output      11111111111

    debounce_time = 0.3
    sample_frequency = 10
    low_value = 0
    high_value = 1

    debounce = Debounce(debounce_time, sample_frequency, low_value, high_value, high_value)

    debounce.record_low_value()
    assert debounce.current_value() == high_value

    debounce.record_high_value()
    assert debounce.current_value() == high_value

    debounce.record_high_value()
    assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == high_value

    debounce.record_high_value()
    assert debounce.current_value() == high_value

    debounce.record_high_value()
    assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == high_value

    debounce.record_high_value()
    assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == high_value

def test_with_jitter_value_change_to_high():
    # real signal 0000111
    # corrupted   0100111
    # output      0000001

    debounce_time = 0.3
    sample_frequency = 10
    low_value = 0
    high_value = 1

    debounce = Debounce(debounce_time, sample_frequency, low_value, high_value, low_value)

    debounce.record_high_value()
    assert debounce.current_value() == low_value

    debounce.record_low_value()
    assert debounce.current_value() == low_value

    debounce.record_low_value()
    assert debounce.current_value() == low_value

    debounce.record_high_value()
    assert debounce.current_value() == low_value

    debounce.record_high_value()
    assert debounce.current_value() == low_value

    debounce.record_high_value()
    assert debounce.current_value() == high_value

def test_with_jitter_value_change_to_low():
    # real signal 1000000
    # corrupted   1100100
    # output      1111110

    debounce_time = 0.3
    sample_frequency = 10
    low_value = 0
    high_value = 1

    debounce = Debounce(debounce_time, sample_frequency, low_value, high_value, high_value)

    debounce.record_high_value()
    assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == high_value

    debounce.record_high_value()
    assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == high_value

    debounce.record_low_value()
    assert debounce.current_value() == low_value

def test_with_jitter_many_value_changes():
    # real signal 0000111111110000000111111100000000011111111110000000000111111100000
    # corrupted   0100111011011001000011011010001001011100101111000100010111011100010
    # output      0000001111111111100000001111100000000111111111110000000001111111000
    pass
