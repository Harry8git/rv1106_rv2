#!/bin/sh
# Create cma and cma-uncached symlinks if reserved heap exists
if [ -e /dev/dma_heap/reserved ]; then
    [ ! -e /dev/dma_heap/cma ] && ln -s /dev/dma_heap/reserved /dev/dma_heap/cma
    [ ! -e /dev/dma_heap/cma-uncached ] && ln -s /dev/dma_heap/reserved /dev/dma_heap/cma-uncached
fi
