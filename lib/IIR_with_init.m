function data = IIR_with_init(data, index, init_value)

n = size(data, 1);

data(1, :) = data(1, :) * index + init_value * (1 - index);

for i = 2 : n
	data(i, :) = data(i, :) * index + data(i-1, :) * (1 - index);
end;
