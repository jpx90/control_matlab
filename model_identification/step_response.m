function m = step_response(iir_index, length)

addpath('../lib');

m = ones(length, 1);
m = IIR_with_init(m, iir_index, 0);

end
