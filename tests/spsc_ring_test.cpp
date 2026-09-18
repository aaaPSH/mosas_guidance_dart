#include <mosas/runtime/spsc_ring.hpp>

#include <cassert>

namespace {

void test_spsc_ring_preserves_order_and_rejects_full() {
    mosas::runtime::SpscRing<int> ring(2);
    assert(ring.try_push(10));
    assert(ring.try_push(20));
    assert(!ring.try_push(30));

    int value = 0;
    assert(ring.try_pop(&value) && value == 10);
    assert(ring.try_pop(&value) && value == 20);
    assert(!ring.try_pop(&value));
}

void test_spsc_ring_rejects_null_output() {
    mosas::runtime::SpscRing<int> ring(2);
    assert(ring.try_push(1));
    assert(!ring.try_pop(nullptr));
}

}  // namespace

int main() {
    test_spsc_ring_preserves_order_and_rejects_full();
    test_spsc_ring_rejects_null_output();
    return 0;
}
