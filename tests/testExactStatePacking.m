classdef testExactStatePacking < matlab.unittest.TestCase
    methods (Test)
        function preservesNestedValuesAndShapes(testCase)
            a = struct('x',single([1;2]),'empty',zeros(0,3), ...
                'flag',true,'text','same','nested',struct('time',0,'value',[NaN,Inf]), ...
                'varying',[1,2],'optional',struct());
            b = a;
            b.x = single([3;4]);
            b.flag = false;
            b.nested.time = 0.01;
            b.varying = 3;
            b.optional = struct('exception','synthetic');
            expected = {a;b};
            actual = unpack_exact_state_rows(pack_exact_state_rows(expected));
            testCase.verifyEqual(actual,expected);
            testCase.verifyClass(actual{2}.x,'single');
            testCase.verifySize(actual{1}.empty,[0,3]);
        end
        function rejectsUnknownLayout(testCase)
            testCase.verifyError(@() unpack_exact_state_rows(struct('layout','unknown')), ...
                'unpack_exact_state_rows:Layout');
        end
        function preservesSignedZero(testCase)
            rows = {struct('x',0);struct('x',-1/Inf)};
            actual = unpack_exact_state_rows(pack_exact_state_rows(rows));
            testCase.verifyEqual(1/actual{1}.x,Inf);
            testCase.verifyEqual(1/actual{2}.x,-Inf);
        end
    end
end
