function init( model, par )
	-- �v�Z�ΏۂƂȂ�body��joint�̎擾
	target_body = model:find_body( scone.target_body )
	subject_assist = model:find_body( "calcn_l" )  -- �l�H�؂̋쓮�^�C�~���O���Ǘ����邽�߂̐g�̕��ʁi�����̂����Ɛڒn�ŋ쓮�j
	joint_back = model:find_joint( "back" )
	joint_hip = model:find_joint( "hip_r" )
	joint_knee = model:find_joint( "knee_r" )
	
	-- PGM���t���ʒu�̎擾
	distance_from_hip = ( scone.distance_from_hip and par:create_from_string( "distance_from_hip", scone.distance_from_hip ) ) / 1000  -- �P�ʂ�[mm]����[m]�ɕϊ�
	distance_from_knee = ( scone.distance_from_knee and par:create_from_string( "distance_from_knee", scone.distance_from_knee ) ) / 1000
	
	-- ���͕ϐ��̎擾
	contraction_time = ( scone.contraction_time and par:create_from_string( "contraction_time", scone.contraction_time ) ) / 1000  -- �P�ʂ�[msec]����[sec]�̕ϊ�
	delay_time = ( scone.delay_time and par:create_from_string( "delay_time", scone.delay_time ) ) / 1000
	target_length = scone.PGM_length and par:create_from_string( "PGM_length", scone.PGM_length )  -- �g�p����l�H�؂́i�ő�j���� [mm]
	pressure = scone.pressure and par:create_from_string( "pressure", scone.pressure )  -- ���͈��� [kPa]
	num_PGM = scone.num_PGM and par:create_from_string( "num_PGM", scone.num_PGM )
	assist_start_gaitcycle = scone.assist_start_gaitcycle and par:create_from_string( "assist_start_gaitcycle", scone.assist_start_gaitcycle )  -- �A�V�X�g���J�n����܂ł̑ҋ@���� [s]
	
	-- �x�N�g���̐V�K�쐬
	pos_1 = vec3:new( 0, 0, 0 )  -- PGM�̎��t���ʒu�i�Ҋ֐�-���́j
	pos_2 = vec3:new( 0, 0, 0 )  -- PGM�̎��t���ʒu�i�Ҋ֐�-�G�j
	position = vec3:new( 0, 0, 0 )  -- �Ҋ֐߂���l�H�؂ɂ��낵�������̌�_���W	
	
	-- �l�H�؂̏��
	length_data = { 200, 220, 240, 260, 280, 300 }  -- �J�^���O����(�ő咷��)
	original_length_data = { 160, 170, 180, 200, 210, 220 }	  -- ���R��
	PGM_length = find_original_length( target_length, length_data )
	
	-- �l�H�؂���������͂��v�Z���邽�߂̌W���̐ݒ�
	define_coefficient()
	
	
	-- �e�퐔�l�̏����l�ݒ�
	is_swing = false  -- �����Ɨ��n�����o����t���O
	is_contracted = false  -- ���̗V�r�ň�񂾂��A�V�X�g�����{���邽�߂̃t���O
	command = 0  -- �l�H�؂��쓮���߂̐M�����Ǘ�
	liftoff = 0  -- �����Ɛڒn���m�F���邽�߂̃t���O
	
	F_spring = 0  -- �l�H�؂̂΂˗v�f�̗�
	F_damper = 0  -- �l�H�؂̃_���p�v�f�̗�
	T_ratio = 0   -- ���k�J�n���痧���オ��܂ł̊���
	
	current_time = 0
	start_time = 0
	current_length = PGM_length
	moment_arm = 0
	force_gain = 0
	
	step_count_l = 0
end
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
function find_original_length( target, data )  -- �g�p����l�H�؂̎��R�����擾����֐�
	for i, length in ipairs(data) do
		if length == target then
			PGM_length = original_length_data[i]
			index = i
		end
	end
	
	return PGM_length
end


function select_values( dataset, num_select )  -- �f�[�^�Z�b�g�z�񂩂�g�p����l�H�؂̒����ɑΉ������f�[�^���擾����֐�
	local index_start = num_select * ( index - 1 ) + 1
	local index_end = num_select * index
	local subset = {}
	
	local j = 1
	for i = index_start, index_end do
		subset[j] = dataset[i]
		j = j + 1
	end
	
	return subset
end


function define_coefficient()  -- �e��W���̐ݒ���s�����߂̊֐�
	K = {}  -- �΂˗v�f�̌W��, K[1] + K[2]*x + K[3]*P + K[4]*x^2 + K[5]*x*P + K[6]*x^3 + K[7]*x^2*P
	local K_dataset = {
	-0.06656,	0.1135,	-0.0003225,	-0.006836,	0.002633,	0.0001998,	1.94E-05,
	-0.5491,	0.1118,	0.001977,		-0.004679,	0.002034,	7.76E-05,		2.92E-05,
	-0.7786,	0.1516,	0.002805,		-0.006071,	0.001832,	8.62E-05,		2.50E-05,
	-0.5973,	0.1159,	0.002666,		-0.004492,	0.001871,	7.20E-05,		1.94E-05,
	-0.5686,	0.1208,	0.001535,		-0.004858,	0.001765,	6.74E-05,		1.72E-05,
	-1.023,		0.1523,	0.003715,		-0.005096,	0.001439,	5.85E-05,		1.65E-05
	}
	K = select_values( K_dataset, 7 )	
	
	D_pos = {}  -- �_���p�v�f�̌W��, (1-exp(-(D[1]+D[2]*P)*v))*((D[3]+D[4]*P)*v+(D[5]+D[6]*P)))
	local D_pos_dataset ={
	0.06159,	0.005398,		0.001339,		-0.00006405,	0.05388,	0.005718,
	0.18,			0.0008367,	-0.002445,	-0.00004965,	0.1311,		0.005098,
	0.08902,	0.0008481,	-0.001893,	-0.00004507,	0.1009,		0.00477,
	0.2947,		0.0002198,	-0.00302,		-0.00005188,	0.08886,	0.004923,
	0.8464	,		-0.0008955,	0.0002279,	-0.00006722,	0.01791,	0.004206,
	0.8047,		-0.0004249,	-0.0001971,	-0.00005298,	0.04274,	0.003788
	}
	D_pos = select_values( D_pos_dataset, 6 )	
	
	D_neg = {}  -- �_���p�v�f�̌W��, (1-exp(-(D[1]+D[2]*P)*v))*((D[3]+D[4]*P)*v+(D[5]+D[6]*P)))
	local D_neg_dataset = {
	-0.003743,	-0.0003437,		0.005425,		-0.0001683,		0.4959,			-0.006305,
	-0.1,				-0.0003891,		-0.002221,	-0.0001054,		-0.01312,		-0.002274,
	-0.1476,		-0.00007419,	-0.001538,	-0.0001001,		-0.002212,	-0.002128,
	-0.007828,	-0.0004975,		0.001498,		-0.000121,		0.2228,			-0.003537,
	-0.01137,		-0.0005959,		-0.003117,	-0.00008256,	-0.01915,		-0.00204,
	-0.1086,		0.0000665,		-0.0008697,	-0.00009687,	0.00838,		-0.002435
	}
	D_neg = select_values( D_neg_dataset, 6 )	
	
	C = {}  -- �����オ�蕔���̌W��, 1/(1+exp(-(C[1]+C[2]*P)*(t-(C[3]+C[4]*P))))
	local C_dataset = {
	34.01,		-0.02133,		0.02511,	0.0001557,
	33.25,		-0.03275,		0.02343,	0.0002228,
	18.49,		0.03408,		0.06893,	0.00007875,
	23.3,		0.01438,		0.05477,	0.00009282,
	35.61,		-0.02532,		0.05437,	0.00003025,
	34.85,		-3.27E-02,	0.0111,		0.0003496
	}
	C = select_values( C_dataset, 4 )
end


function detect_liftoff()  -- �V�r�����o���邽�߂̊֐�
	if ( subject_assist:contact_force().y > 200 ) and ( subject_assist:contact_force().x < 170 ) then
		is_swing = true
		is_contracted = false
		start_time = current_time
		liftoff = 1
		step_count_l = step_count_l + 1
		
	end
end

function detect_landing()	  -- ���r�����o���邽�߂̊֐�
	if subject_assist:contact_force().y < 1 then
		is_swing = false
		liftoff = 0
		
	end
end



function calculate_positions()  -- �l�H�؂̎��t���ʒu����Ɍv�Z
	-- �e�֐߈ʒu�̎擾
	pos_back = joint_back:pos()
	pos_hip = joint_hip:pos()
	pos_knee = joint_knee:pos()
	
	-- ���[�����g�A�[���̌v�Z
	pos_1 = pos_hip + ( pos_back - pos_hip ) / math.sqrt( ( pos_back.x - pos_hip.x )^2 + ( pos_back.y - pos_hip.y )^2 + ( pos_back.z - pos_hip.z )^2 ) * distance_from_hip  -- ���̑��̎��t���ʒu�̃O���[�o�����W
	pos_2 = pos_knee + ( pos_hip - pos_knee ) / math.sqrt( ( pos_hip.x - pos_knee.x )^2 + ( pos_hip.y - pos_knee.y )^2 + ( pos_hip.z - pos_knee.z )^2 ) * distance_from_knee  -- ��ڑ��̎��t���ʒu�̃O���[�o�����W
	
	-- �l�H�؂̒������v�Z	
	current_length = math.sqrt( ( pos_1.x - pos_2.x )^2 + ( pos_1.y - pos_2.y )^2 + ( pos_1.z - pos_2.z )^2 ) * 1000	
end



function calculate_moment_arm()  -- ���[�����g�A�[�����v�Z���邽�߂̊֐�
	-- �Ҋ֐ߑ��̎��t���ʒu���烂�[�����g�A�[���܂ł̒����́A�l�H�؂̒����ɑ΂���䗦
	local ratio = ( ( pos_2.x - pos_1.x )*( pos_hip.x - pos_1.x ) +  ( pos_2.y - pos_1.y )*( pos_hip.y - pos_1.y ) + ( pos_2.z - pos_1.z )*( pos_hip.z - pos_1.z ) )
	/ ( ( pos_2.x - pos_1.x )^2 + ( pos_2.y - pos_1.y )^2 + ( pos_2.z - pos_1.z )^2 )
	
	position = ( 1 - ratio )*pos_1 + ratio*pos_2  -- �Ҋ֐߂���̐����Ɛl�H�؂̌�_�̃O���[�o�����W
	moment_arm = math.sqrt( ( position.x - pos_hip.x )^2 +  ( position.y - pos_hip.y )^2 +  ( position.z - pos_hip.z )^2 )
end



function artificial_muscle_force(t)  -- �l�H�؂���������͂��v�Z���邽�߂̊֐�	
	-- �l�H�؂̐L��, �L�ё��x���v�Z
	x = current_length - PGM_length
	v = ( current_length - previous_length )/( current_time - previous_time )	
	
	-- �l�H�؂̈����͂��v�Z
	F_spring = K[1] + K[2]*x + K[3]*pressure + K[4]*x^2 + K[5]*x*pressure + K[6]*x^3 + K[7]*x^2*pressure  -- [N]
	if v > 0 then
		F_damper = (1-math.exp(-(D_pos[1]+D_pos[2]*pressure)*v))*((D_pos[3]+D_pos[4]*pressure)*v+(D_pos[5]+D_pos[6]*pressure))  -- [N]
	else
		F_damper = (1-math.exp(-(D_neg[1]+D_neg[2]*pressure)*v))*((D_neg[3]+D_neg[4]*pressure)*v+(D_neg[5]+D_neg[6]*pressure))
	end  -- 1/(1+exp(-(C[1]+C[2]*P)*(t-(C[3]+C[4]*P))))
	T_ratio = 1/( 1 + math.exp( -(C[1]+C[2]*pressure)*(t-(C[3]+C[4]*pressure)) ) )	
	
	-- �v�Z���ʂ𑫂����킹���l�H�؂̗\������
	force_gain = num_PGM * ( F_spring + F_damper ) * T_ratio
end



function assist_controller( model )  -- �A�V�X�g�̐�����s���֐�
	
	if command == 0 then  -- �͂��������Ă��Ȃ��ꍇ
		
		if ( current_time - start_time ) > delay_time and not is_contracted then  -- �܂����k�����s����ĂȂ��ꍇ�@���@�x�����ԕ��ҋ@�����������ꍇ, ���k�J�n
			start_time = current_time
			command = 1
			
		end
	end
	
	if command == 1 then  -- �͂��������Ă���ꍇ(�A�V�X�g��)
		
		-- ���O�̃t���[���̒l��ۑ�
		moment_arm_old = moment_arm
		force_gain_old = force_gain
		
		-- �V����moment_arm��force_gain���v�Z
		calculate_moment_arm()
		artificial_muscle_force( current_time - start_time )
		
		-- ���k���Ԃ��o�߂����ꍇ, ���k�I��
		if ( current_time - start_time ) > contraction_time then
			is_contracted = true
			command = 0
			force_gain = 0
			F_spring = 0
			F_damper = 0
			T_ratio = 0
			
		end
		
		-- �A�V�X�g���[�����g�̕t��
		assist_moment = force_gain * moment_arm
		target_body:add_external_moment( 0, 0, ( force_gain * moment_arm ) - ( force_gain_old * moment_arm_old ) )
	end
	
end
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
function update( model )
	current_time = model:time()  -- ���݂̎������擾
	
	calculate_positions()  -- �l�H�؂̎��t���ʒu����Ɍv�Z
	
	if is_swing then  -- �V�r��
		detect_landing()	
	else  -- ���r��
		detect_liftoff()
	end
	
	if step_count_l >= assist_start_gaitcycle then
		assist_controller( model )
	end
	
	previous_time = current_time  -- 1�t���[���O�̎������L�^
	previous_length = current_length
	return false;
end



function store_data( frame )
	frame:set_value( "moment_arm_r", moment_arm )
	frame:set_value( "assist_moment_r", assist_moment )
	
	frame:set_value( "F_spring_r", F_spring )
	frame:set_value( "F_damper_r", F_damper )
	frame:set_value( "T_ratio_r", T_ratio )
	
	frame:set_value( "command_r", command )
	
	frame:set_value( "step_count_l", step_count_left )
end
