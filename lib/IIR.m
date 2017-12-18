function data = IIR(data, index)

n = size(data, 1);

for i = 2 : n
	data(i, :) = data(i, :) * index + data(i-1, :) * (1 - index);
end;
