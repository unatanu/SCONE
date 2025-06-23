function init( model, par, side )
	subject_landing_r = model:find_body( "calcn_r" )
	subject_landing_l = model:find_body( "calcn_l" )
	
	step_count = -1  -- �����𐔂��邽�߂̕ϐ�
	is_stand_r = false
	is_stand_l = false
end

function update( model )
	-- �E���̏����͂������̏����͂��傫���Ȃ����Ƃ�
	if ( not is_stand_r ) and ( subject_landing_r:contact_force().y > subject_landing_l:contact_force().y ) then
		step_count = step_count + 1
		is_stand_r = true
		is_stand_l = false
	
	-- �����̏����͂��E���̏����͂��傫���Ȃ����Ƃ�
	elseif ( not is_stand_l ) and ( subject_landing_l:contact_force().y > subject_landing_r:contact_force().y ) then
		step_count = step_count + 1
		is_stand_r = false
		is_stand_l = true
	end	
	
	return false
end

function store_data( frame )
	frame:set_value( "step_count", step_count )
end
