function automatic int dma_log2(input int val); begin
    int n;
    n = 0;
    while (2 ** n < val) begin
        ++n;
    end
    dma_log2 = n;
end
endfunction
