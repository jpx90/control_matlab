function v = abs_constrain(v, max_abs)

if v > max_abs
	v = max_abs;
elseif v < -max_abs
	v = -max_abs;
end;

end