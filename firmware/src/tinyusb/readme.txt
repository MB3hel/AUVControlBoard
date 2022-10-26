Reference family.mk (hw/bsp/samd51) and board.mk for ItsyBitsy M4 to know what to change. The changes that have been made are listed below.

- All non-samd51 files removed from portable
- bsp folder added with family.c and board.h
- family.c modified to **NOT** use ASF4
