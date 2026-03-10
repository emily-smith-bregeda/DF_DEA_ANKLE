function [Filtered_Tracking_LCS, debug] = filterTransform4x4(Tracking_LCS, prefix, jointSelection, fs, filterChoice, varargin)
%FILTERTRANSFORM4X4 Filter 4x4xN tracking transforms inside Tracking_LCS.(prefix).<fields>
%
% Required inputs:
%   Tracking_LCS   struct
%   prefix         string/char (mandatory)
%   jointSelection scalar >= 1 (mandatory)
%   fs             sampling frequency (Hz)
%   filterChoice   1..7 or string:
%                 1 DBWavelet, 2 PZWavelet, 3 Butterworth, 4 Gauss,
%                 5 Mean, 6 SLERP, 7 ROUT
%
% Assumptions:
%   - All tracking matrices are 4x4xN
%   - Rotations are handled using MATLAB quaternion class
%
% Optional name-value pairs:
%   'CutoffHz'      (default 12)
%   'ButterOrder'   (default 4)   % matches your old FilterTracking.m
%   'WaveletName'   (default 'db4')
%   'WaveletLevel'  (default 2)
%   'Iterations'    (default 2)
%   'Q'             (default 0.01) % ROUT FDR parameter
%   'Fields'        (default auto based on jointSelection)
%   'KeepMissing'   (default true)
%   'RotType'       (default "frame") % passed to quaternion(rotmat, RotType) + rotmat(quat, RotType)
%
% Notes:
%   - quaternion(RM,"rotmat",RotType) creates quats from 3x3xN matrices. [1](https://www.mathworks.com/help/robotics/ref/quaternion.html)
%   - rotmat(q,RotType) converts back to 3x3xN. [3](https://www.mathworks.com/help/robotics/ref/quaternion.rotmat.html)
%   - exp/log operate on quaternion arrays. [2](https://www.mathworks.com/help/nav/ref/quaternion.exp.html)
%   - left division A\B gives inv(A)*B for quaternion scalars. [4](https://www.mathworks.com/help/driving/ref/quaternion.mldivide.html)
%   - mtimes (A*B) supports quaternion multiplication when one operand is scalar. [5](https://www.mathworks.com/help/fusion/ref/quaternion.mtimes.html)

% -------------------- validate required inputs --------------------
if ~isstruct(Tracking_LCS)
    error('Tracking_LCS must be a struct.');
end

prefix = string(prefix);
if strlength(prefix) == 0
    error('prefix must be a non-empty string/char.');
end

if ~(isscalar(jointSelection) && jointSelection >= 1)
    error('jointSelection must be a scalar >= 1.');
end

if ~(isscalar(fs) && fs > 0)
    error('fs must be a positive scalar (Hz).');
end

% Map filterChoice if string
if ischar(filterChoice) || isstring(filterChoice)
    filterChoice = local_mapFilterChoice(filterChoice);
end

% -------------------- parse optional settings --------------------
p = inputParser;
p.addParameter('CutoffHz', 2, @(x)isscalar(x)&&x>0);
p.addParameter('ButterOrder', 4, @(x)isscalar(x)&&x>=1);
p.addParameter('WaveletName', 'db4', @(s)ischar(s)||isstring(s));
p.addParameter('WaveletLevel', 2, @(x)isscalar(x)&&x>=1);
p.addParameter('Iterations', 2, @(x)isscalar(x)&&x>=1);
p.addParameter('Q', 0.01, @(x)isscalar(x)&&x>0);
p.addParameter('Fields', {}, @(c)iscell(c)||isstring(c));
p.addParameter('KeepMissing', true, @(x)islogical(x)&&isscalar(x));
p.addParameter('RotType', "frame", @(s)ischar(s)||isstring(s)); % "frame" or "point"
p.parse(varargin{:});
opt = p.Results;
rotType = string(opt.RotType);

% -------------------- choose fields to filter --------------------
if ~isempty(opt.Fields)
    fields = cellstr(opt.Fields);
else
    fields = {'TibiaTT','TalusTT'};
    if jointSelection > 1
        fields = [fields, {'TalusST','CalcaneusST'}];
        if jointSelection >= 3
            fields = [fields, {'TalusTN','NavicularTN'}];
        end
        if jointSelection >= 4
            fields = [fields, {'CalcaneusCC','CuboidCC'}];
        end
    end
end

% -------------------- validate prefix exists --------------------
if ~isfield(Tracking_LCS, prefix)
    error('Tracking_LCS does not contain prefix: %s', prefix);
end

% -------------------- filter each 4x4xN field --------------------
Filtered_Tracking_LCS = Tracking_LCS;
dbgFields = struct();

for i = 1:numel(fields)
    fld = fields{i};

    if ~isfield(Tracking_LCS.(prefix), fld)
        if opt.KeepMissing
            continue
        else
            error('Missing field: Tracking_LCS.%s.%s', prefix, fld);
        end
    end

    Traw = Tracking_LCS.(prefix).(fld);
    local_assert4x4xN(Traw, sprintf('Expected 4x4xN for Tracking_LCS.%s.%s', prefix, fld));

    [Tfilt, dbgCore] = local_filter4x4xN_quat(Traw, fs, filterChoice, opt, rotType);
    Filtered_Tracking_LCS.(prefix).(fld) = Tfilt;

    dbgFields.(fld) = dbgCore;
end

debug = struct();
debug.mode = "struct-only-quaternion";
debug.prefix = prefix;
debug.jointSelection = jointSelection;
debug.fields = fields;
debug.filterChoice = filterChoice;
debug.rotType = rotType;
debug.perField = dbgFields;

end

% =====================================================================
% CORE: Filter one 4x4xN sequence using quaternion class
% =====================================================================
function [Tfilt, debug] = local_filter4x4xN_quat(Traw, fs, filterChoice, opt, rotType)

N = size(Traw,3);
dt = 1/fs;
Time = (0:N-1)' * dt;

R = Traw(1:3,1:3,:);                 % 3x3xN
t = squeeze(Traw(1:3,4,:)).';        % Nx3

% Create quaternion array from rotation matrices (N-by-1 quaternion array). [1](https://www.mathworks.com/help/robotics/ref/quaternion.html)
q = normalize(quaternion(R, "rotmat", rotType));
q0 = q(1);

% Convenience flags: only compute log-velocity when needed
needsVel  = ismember(filterChoice, [1 2 3 5 7]);  % DB, PZ, Butter, Mean, ROUT (we'll do ROUT in log space)
needsDiff = (filterChoice == 4);                  % Gauss uses neighbor diffs

% -------------------------------------------------------------
% Build log-space signals when needed:
%   rel = q0 \ q  (inv(q0) * q) for quaternion scalars [4](https://www.mathworks.com/help/driving/ref/quaternion.mldivide.html)
%   log(rel) -> pure quaternion increments; use vector part as Euclidean coords
% -------------------------------------------------------------
if needsVel
    qRel = q0 \ q;          % inv(q0)*q for each element  [4](https://www.mathworks.com/help/driving/ref/quaternion.mldivide.html)
    qLog = log(qRel);       % quaternion log (overloaded) [2](https://www.mathworks.com/help/nav/ref/quaternion.exp.html)
    parts = compact(qLog);  % Nx4 [w x y z]  (commonly used with quaternion arrays) [6](https://www.mathworks.com/help/fusion/ug/lowpass-filter-orientation-using-quaternion-slerp.html)
    rotvec = parts(:,2:4);  % use only vector part (scalar part should be ~0)
    vel = rotvec / dt;      % Nx3
end

if needsDiff
    qDiff = zeros(N,1,"quaternion");
    qDiff(1) = quaternion(0,0,0,0);
    for k = 1:N-1
        qRel_k = q(k) \ q(k+1);   % inv(q(k))*q(k+1)
        qDiff(k+1) = log(qRel_k); % quaternion
    end
end

% -------------------------------------------------------------
% Apply chosen filter
% -------------------------------------------------------------
switch filterChoice

    case 1  % DBWavelet (velocity domain)
        it = opt.Iterations;
        vFilt = vel;
        tFilt = t;
        for r = 1:it
            vFilt = wdenoise(vFilt, 'Wavelet', char(opt.WaveletName), opt.WaveletLevel);
            tFilt = wdenoise(tFilt, 'Wavelet', char(opt.WaveletName), opt.WaveletLevel);
        end
        qFilt = local_reconstruct_from_rotvec(q0, vFilt*dt);

    case 2  % PZWavelet (velocity domain, uses your WaveletFilter)
        vFilt = (WaveletFilter(vel', dt, 2, 2, 1))';
        tFilt = (WaveletFilter(t',   dt, 2, 1, 2))';
        qFilt = local_reconstruct_from_rotvec(q0, vFilt*dt);

    case 3  % Butterworth (velocity domain)
        fc = opt.CutoffHz;
        [b,a] = butter(opt.ButterOrder, fc/(fs/2), 'low');
        vFilt = filtfilt(b,a,vel);
        tFilt = filtfilt(b,a,t);
        qFilt = local_reconstruct_from_rotvec(q0, vFilt*dt);

    case 4  % Gauss (quaternion-domain smoothing + Gaussian FIR on translation)
        it = opt.Iterations;
        w = [1/16 4/16 6/16 4/16 1/16];

        qFilt = q;
        tFilt = t;

        for r = 1:it
            % (Optional improvement) recompute diffs from current iterate:
            qDiff = zeros(N,1,"quaternion");
            qDiff(1) = quaternion(0,0,0,0);
            for k = 1:N-1
                qRel_k = qFilt(k) \ qFilt(k+1);
                qDiff(k+1) = log(qRel_k);
            end

            qNew = qFilt;
            qNew(1:2) = qFilt(1:2);
            for j = 3:N-1
                delta = (1/16) * (-qDiff(j-2) - 5*qDiff(j-1) + 5*qDiff(j) + qDiff(j+1));
                qNew(j) = qFilt(j) * exp(delta);  % scalar*scalar multiply OK [5](https://www.mathworks.com/help/fusion/ref/quaternion.mtimes.html)[2](https://www.mathworks.com/help/nav/ref/quaternion.exp.html)
            end
            qNew(N) = qFilt(N);

            qFilt = normalize(qNew);
            tFilt = filtfilt(w, 1, tFilt);
        end

    case 5  % Mean (velocity domain)
        it = opt.Iterations;
        w = (1/5)*ones(1,5);
        vFilt = vel;
        tFilt = t;
        for r = 1:it
            vFilt = filtfilt(w, 1, vFilt);
            tFilt = filtfilt(w, 1, tFilt);
        end
        qFilt = local_reconstruct_from_rotvec(q0, vFilt*dt);

    case 6  % SLERP (orientation domain)
        % MATLAB provides slerp/dist on quaternion arrays. [7](https://www.mathworks.com/help/vision/ref/quaternion.slerp.html)
        hrange = 0.4;
        hbias  = 0.5;
        low  = max(min(hbias - (hrange./2), 1), 0);
        high = max(min(hbias + (hrange./2), 1), 0);
        hrangeLimited = high - low;

        y = q(1);
        qF1 = q;
        qF1(1) = y;

        for k = 2:N
            x = q(k);
            d = dist(y, x);
            a = (d/pi)*hrangeLimited + low;
            y = slerp(y, x, a);
            qF1(k) = y;
        end

        y = qF1(end);
        qFilt = qF1;
        qFilt(end) = y;

        for k = N:-1:1
            x = qF1(k);
            d = dist(y, x);
            a = (d/pi)*hrangeLimited + low;
            y = slerp(y, x, a);
            qFilt(k) = y;
        end

        qFilt = normalize(qFilt);

        % translation: 4th-order Butterworth
        fc = opt.CutoffHz;
        [b,a] = butter(4, fc/(fs/2), 'low');
        tFilt = filtfilt(b,a,t);

    case 7  % ROUT (robust polynomial fit) in log space (more rotation-consistent)
        Q = opt.Q;

        modelstr = 'y ~ b1*x^3 + b2*x^2 + b3*x + b4';
        modelfun = @(b,x) b(1)*x.^3 + b(2)*x.^2 + b(3)*x + b(4);
        opts2 = statset('nlinfit');
        opts2.RobustWgtFun = 'cauchy';

        b0 = [0;0;0;0];
        K = numel(b0);
        DoF = N-K;

        % data: 3 rotvec components + 3 translations = Nx6
        data = [rotvec t];
        dataFilt = zeros(size(data));

        for col = 1:size(data,2)
            mdl = fitnlm(Time, data(:,col), modelstr, b0, 'Options', opts2);

            res = abs(table2array(mdl.Residuals(:,1)));
            idx = (1:N)';

            P68  = prctile(res, 68);
            RSDR = P68*(N/DoF);

            [resS, ord] = sort(res,'ascend');
            idxS = idx(ord);

            outlierIdx = [];
            for j = round(0.70*N):N
                alpha = (Q*(N-(j-1)))/N;
                tval  = resS(j)/RSDR;
                p2 = 2*tcdf(-abs(tval), DoF);
                if p2 < alpha
                    outlierIdx = idxS(j:end);
                    break
                end
            end

            y = data(:,col);
            tt = Time;
            if ~isempty(outlierIdx)
                y(outlierIdx)  = [];
                tt(outlierIdx) = [];
            end

            mdl2 = fitnlm(tt, y, modelfun, b0);
            coeff = table2array(mdl2.Coefficients(:,1));
            dataFilt(:,col) = polyval(coeff, Time);
        end

        rvFilt = dataFilt(:,1:3);
        tFilt  = dataFilt(:,4:6);
        qFilt  = local_reconstruct_from_rotvec(q0, rvFilt);

    otherwise
        error('Unknown filterChoice.');
end

% -------------------------------------------------------------
% Convert back to 4x4xN
%   rotmat(qFilt, rotType) -> 3x3xN [3](https://www.mathworks.com/help/robotics/ref/quaternion.rotmat.html)
% -------------------------------------------------------------
Rfilt = rotmat(qFilt, rotType);

Tfilt = zeros(4,4,N);
Tfilt(4,4,:) = 1;
for k = 1:N
    Tfilt(1:3,1:3,k) = Rfilt(:,:,k);
    Tfilt(1:3,4,k)   = tFilt(k,:).';
end

debug = struct();
debug.fs = fs;
debug.dt = dt;
debug.N  = N;
debug.Time = Time;
debug.filterChoice = filterChoice;

end

% =====================================================================
% Reconstruct quaternion trajectory from filtered rotation-vectors (log space)
%   qFilt(k) = q0 * exp( [0, rv(k,:)] )
% =====================================================================
function qOut = local_reconstruct_from_rotvec(q0, rv)
% rv: Nx3 corresponds to vector part of log(q0\q)
N = size(rv,1);
pure = quaternion([zeros(N,1), rv]);  % Nx4 parts -> Nx1 quaternion [1](https://www.mathworks.com/help/robotics/ref/quaternion.html)
qStep = exp(pure);                    % quaternion exp [2](https://www.mathworks.com/help/nav/ref/quaternion.exp.html)
qOut  = q0 * qStep;                   % scalar*array multiplication [5](https://www.mathworks.com/help/fusion/ref/quaternion.mtimes.html)
qOut  = normalize(qOut);
end

% =====================================================================
% Helpers
% =====================================================================
function c = local_mapFilterChoice(name)
name = lower(string(name));
switch name
    case {"1","dbwavelet","db","db wavelet"}, c = 1;
    case {"2","pzwavelet","pz","pz wavelet"}, c = 2;
    case {"3","butterworth","butter"},        c = 3;
    case {"4","gauss","gaussian"},            c = 4;
    case {"5","mean","moving average"},       c = 5;
    case {"6","slerp"},                       c = 6;
    case {"7","rout"},                        c = 7;
    otherwise, error('Unknown filter name: %s', name);
end
end

function local_assert4x4xN(T, msg)
if ndims(T) ~= 3 || size(T,1) ~= 4 || size(T,2) ~= 4
    error(msg);
end
end