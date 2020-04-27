from matplotlib import pyplot as plt
import numpy as np
from geometry.Point import Point
from geometry.Vector import Vector

waypoints = np.array([
    [-0.000458,-0.000361],
    [0.142214,0.270564],
    [0.223221,0.573814],
    [0.676189,1.493873],
    [0.932008,1.659184]
])

x, y = waypoints.T
fig = plt.figure(figsize=(8, 8))
plt.gca().set_aspect('equal', adjustable='box')
plt.scatter(x, y, marker='+')
plt.pause(0.05)

real_set = np.array([
    [-0.001072,-0.001256, -0.001072, -0.001256 ,0.000614, 0.000895],
    [0.001204,0.000783, 0.001204, 0.000783 ,0.14101, 0.26978100000000005],
    [-4e-05,-1e-05, -4e-05, -1e-05 ,0.14225400000000002, 0.27057400000000004],
    [-0.001464,-0.00044, -0.001464, -0.00044 ,0.143678, 0.271004],
    [-0.002611,-0.002084, -0.002611, -0.002084 ,0.144825, 0.272648],
    [-0.000326,-0.00121, -0.000326, -0.00121 ,0.14254, 0.271774],
    [-0.001298,-0.001993, -0.001298, -0.001993 ,0.143512, 0.27255700000000005],
    [-0.004695,-0.004817, -0.004695, -0.004817 ,0.146909, 0.27538100000000004],
    [0.001389,0.002954, 0.001389, 0.002954 ,0.140825, 0.26761],
    [-0.002616,-0.001393, -0.002616, -0.001393 ,0.14483000000000001, 0.271957],
    [-0.000421,-0.000592, -0.000421, -0.000592 ,0.142635, 0.271156],
    [-0.002334,-0.006117, -0.002334, -0.006117 ,0.144548, 0.276681],
    [-0.002008,-0.000785, -0.002008, -0.000785 ,0.14422200000000002, 0.271349],
    [-0.00206,-0.003617, -0.00206, -0.003617 ,0.144274, 0.274181],
    [4e-06,0.000428, 4e-06, 0.000428 ,0.14221, 0.27013600000000004],
    [2.4e-05,-0.001062, 2.4e-05, -0.001062 ,0.14219, 0.27162600000000003],
    [-0.004967,-0.004821, -0.004967, -0.004821 ,0.147181, 0.27538500000000005],
    [-0.000453,-0.001095, -0.000453, -0.001095 ,0.14266700000000002, 0.27165900000000004],
    [-0.00088,-0.0037, -0.00088, -0.0037 ,0.143094, 0.274264],
    [-0.001032,-0.003967, -0.001032, -0.003967 ,0.143246, 0.274531],
    [-0.001064,-0.000561, -0.001064, -0.000561 ,0.14327800000000002, 0.271125],
    [0.000842,0.000152, 0.000842, 0.000152 ,0.141372, 0.27041200000000004],
    [-0.003058,0.00073, -0.003058, 0.00073 ,0.145272, 0.269834],
    [-0.002815,-0.001256, -0.002815, -0.001256 ,0.14502900000000002, 0.27182],
    [0.001316,0.001536, 0.001316, 0.001536 ,0.140898, 0.26902800000000004],
    [0.005642,0.007203, 0.005642, 0.007203 ,0.136572, 0.263361],
    [0.006968,0.009646, 0.006968, 0.009646 ,0.135246, 0.26091800000000004],
    [0.007679,0.011036, 0.007679, 0.011036 ,0.13453500000000002, 0.25952800000000004],
    [0.026187,0.032931, 0.029002, 0.03511705369441838 ,0.116027, 0.23763300000000004],
    [0.039472,0.052726, 0.038156, 0.05008367924928718 ,0.102742, 0.21783800000000003],
    [0.041243,0.058286, 0.035601, 0.04867501069900188 ,0.100971, 0.21227800000000002],
    [0.052755,0.076256, 0.045787, 0.06524149661188977 ,0.08945900000000001, 0.19430800000000004],
    [0.076792,0.104368, 0.069113, 0.09584410608388694 ,0.06542200000000001, 0.166196],
    [0.080628,0.15362, 0.054441, 0.10340864990942161 ,0.061586, 0.11694400000000002],
    [0.093742,0.192523, 0.054270000000000006, 0.12817256468241303 ,0.048472, 0.07804100000000003],
     [0.116572,0.217624, 0.075329, 0.17131944221102247 ,0.10664900000000001, 0.35619],
     [0.12911,0.236152, 0.076355, 0.1673445101743154 ,0.094111, 0.337662],
     [0.134039,0.246596, 0.05724699999999999, 0.11818985121412945 ,0.08918200000000001, 0.327218],
     [0.156766,0.268464, 0.07613799999999998, 0.11093917525180225 ,0.06645500000000001, 0.30535000000000007],
     [0.170385,0.282498, 0.076643, 0.09137533486238167 ,0.052835999999999994, 0.291316],
     [0.175962,0.293007, 0.05939000000000001, 0.07091731256197781 ,0.047258999999999995, 0.28080700000000003],
     [0.178981,0.296727, 0.049871, 0.05728517847026543 ,0.04424, 0.277087],
     [0.178767,0.296792, 0.04472800000000002, 0.05019925125484559 ,0.044453999999999994, 0.27702200000000005],
     [0.179195,0.297774, 0.022429000000000004, 0.029489658860141876 ,0.04402600000000001, 0.27604000000000006],
     [0.178793,0.29707, 0.008407999999999999, 0.014571999999999974 ,0.044427999999999995, 0.27674400000000005],
     [0.179115,0.297326, 0.008729999999999988, 0.014827999999999952 ,0.044106000000000006, 0.27648800000000007],
     [0.178724,0.296907, 0.008338999999999985, 0.01440899999999995 ,0.04449700000000001, 0.27690700000000007],
     [0.179879,0.303001, 0.009494000000000002, 0.020502999999999993 ,0.04334199999999999, 0.270813],
     [0.186758,0.31087, 0.016373, 0.028371999999999953 ,0.036462999999999995, 0.26294400000000007],
     [0.187143,0.311282, 0.016757999999999995, 0.028783999999999976 ,0.036078, 0.26253200000000004],
     [0.188074,0.312604, 0.017688999999999983, 0.030105999999999966 ,0.03514700000000001, 0.26121000000000005],
     [0.18689,0.311975, 0.016504999999999992, 0.029476999999999975 ,0.036331, 0.26183900000000004],
     [0.191147,0.319724, 0.020762000000000003, 0.03722599999999998 ,0.03207399999999999, 0.25409000000000004],
     [0.190755,0.319573, 0.02037, 0.03707499999999997 ,0.032465999999999995, 0.25424100000000005],
     [0.199169,0.326601, 0.028784000000000004, 0.04410299999999995 ,0.02405199999999999, 0.24721300000000007],
     [0.199091,0.326599, -7.80000000000225e-05, -2.000000000002e-06 ,0.024130000000000013, 0.24721500000000007],
     [0.197384,0.324832, -0.0017850000000000088, -0.001768999999999965 ,0.025837, 0.24898200000000004],
     [0.19756,0.327756, -0.0016089999999999993, 0.0011550000000000171 ,0.02566099999999999, 0.24605800000000005],
     [0.20023,0.332366, 0.0010609999999999786, 0.00576500000000002 ,0.02299100000000001, 0.24144800000000005],
     [0.204688,0.3369, 0.005518999999999996, 0.010299000000000003 ,0.018532999999999994, 0.23691400000000007],
     [0.202687,0.335243, 0.0035179999999999934, 0.008642000000000039 ,0.020533999999999997, 0.23857100000000003],
     [0.207939,0.340062, 0.00877, 0.013461 ,0.01528199999999999, 0.23375200000000007],
     [0.212855,0.346143, 0.013685999999999976, 0.019542000000000004 ,0.010366000000000014, 0.22767100000000007],
     [0.22443,0.359171, 0.0242, 0.026842468092554417 ,-0.0012089999999999879, 0.21464300000000003],
     [0.230717,0.379109, 0.026028999999999997, 0.037991807273202394 ,-0.007496000000000003, 0.19470500000000007],
     [0.234989,0.39098, 0.032302, 0.05419176415596855 ,-0.011768, 0.18283400000000005],
     [0.252178,0.41524, 0.044239, 0.07852556282149808 ,-0.02895700000000001, 0.15857400000000005],
     [0.256613,0.422374, 0.04375799999999999, 0.07867365449710628 ,-0.03339199999999998, 0.15144000000000002],
     [0.261003,0.421314, 0.036572999999999994, 0.06087486318137342 ,-0.03778199999999998, 0.15250000000000002],
     [0.259965,0.417951, 0.029247999999999996, 0.03961399076851879 ,-0.036744, 0.15586300000000003],
     [0.247572,0.419982, -0.009040999999999993, -0.0023920000000000052 ,-0.024350999999999984, 0.15383200000000002],
     [0.244986,0.417806, -0.01162699999999997, -0.0045680000000000165 ,-0.021765000000000007, 0.15600800000000004],
     [0.256493,0.433118, -0.00011999999999995348, 0.010743999999999976 ,-0.033272000000000024, 0.14069600000000004],
     [0.259144,0.448093, 0.0025310000000000055, 0.025718999999999992 ,-0.03592299999999998, 0.12572100000000003],
     [0.264565,0.455228, 0.007952000000000015, 0.032853999999999994 ,-0.04134399999999999, 0.11858600000000002],
     [0.278343,0.468522, 0.03077100000000002, 0.049933019797964995 ,-0.055122000000000004, 0.10529200000000005],
     [0.277291,0.489374, 0.032305, 0.060742838633775886 ,-0.05407000000000001, 0.08444000000000007],
   [0.284697,0.518447, 0.02820399999999995, 0.06994007787993134 ,-0.061475999999999975, 0.055367000000000055],
   [0.275599,0.541112, 0.016454999999999997, 0.044561022439421394 ,0.40059000000000006, 0.952761],
   [0.282316,0.553721, 0.017751000000000017, 0.0633711877765627 ,0.39387300000000003, 0.940152],
   [0.29457,0.559856, 0.01622699999999999, 0.047199547457536406 ,0.38161900000000004, 0.934017],
   [0.293401,0.560463, 0.016110000000000013, 0.035905090323680056 ,0.382788, 0.93341],
   [0.313316,0.583217, 0.028619000000000006, 0.036623662429675075 ,0.36287300000000006, 0.910656],
   [0.317345,0.591398, 0.041746000000000005, 0.046999301626404755 ,0.35884400000000005, 0.902475],
   [0.340691,0.622933, 0.05837500000000001, 0.07193605429816707 ,0.335498, 0.87094],
   [0.363316,0.63843, 0.06874599999999997, 0.08091805422400533 ,0.31287300000000007, 0.855443],
   [0.389088,0.662923, 0.09568699999999997, 0.10253445578058284 ,0.28710100000000005, 0.83095],
   [0.399857,0.671265, 0.08654100000000003, 0.08576725212582192 ,0.276332, 0.822608],
   [0.414718,0.678563, 0.09737299999999999, 0.08635985146933756 ,0.26147100000000006, 0.81531],
   [0.427572,0.691797, 0.08688099999999999, 0.0688718022806738 ,0.24861700000000003, 0.802076],
   [0.439885,0.700133, 0.07656900000000005, 0.060527095955810184 ,0.23630400000000001, 0.79374],
   [0.44495,0.705709, 0.05586200000000002, 0.042260434619182385 ,0.23123900000000003, 0.788164],
   [0.44614,0.706118, 0.04628299999999996, 0.03617660635559339 ,0.23004900000000006, 0.787755],
   [0.446278,0.706159, 0.03156000000000003, 0.02710979951934267 ,0.22991100000000003, 0.787714],
   [0.446089,0.70589, 0.03137100000000004, 0.02732699999999999 ,0.23010000000000003, 0.787983],
   [0.446535,0.706848, 0.03181700000000004, 0.028285000000000005 ,0.22965400000000002, 0.787025],
   [0.446143,0.706077, 0.031425000000000036, 0.027513999999999927 ,0.23004600000000003, 0.787796],
   [0.446193,0.706382, 0.03147500000000003, 0.027818999999999927 ,0.22999600000000003, 0.787491],
   [0.44593,0.705729, 0.031212000000000018, 0.027166000000000023 ,0.23025900000000005, 0.788144],
   [0.446027,0.70584, 0.03130900000000003, 0.027276999999999996 ,0.23016200000000003, 0.788033],
   [0.446367,0.706626, 0.03164900000000004, 0.02806299999999995 ,0.22982200000000003, 0.787247],
   [0.446285,0.706274, 0.03156700000000001, 0.02771099999999993 ,0.22990400000000005, 0.787599],
   [0.446434,0.706504, 0.03171600000000002, 0.027940999999999994 ,0.22975500000000004, 0.787369],
   [0.445564,0.705109, 0.03084600000000004, 0.02654599999999996 ,0.23062500000000002, 0.788764],
   [0.44593,0.705467, 0.031212000000000018, 0.026903999999999928 ,0.23025900000000005, 0.788406],
   [0.451735,0.706098, 0.03701700000000002, 0.027534999999999976 ,0.22445400000000004, 0.787775],
   [0.452602,0.706801, 0.03788400000000003, 0.028237999999999985 ,0.22358700000000004, 0.787072],
   [0.452537,0.707199, 0.03781900000000005, 0.028635999999999995 ,0.22365200000000002, 0.786674],
   [0.452423,0.706745, 0.037705000000000044, 0.02818199999999993 ,0.22376600000000002, 0.787128],
   [0.452317,0.706749, 0.03759900000000005, 0.028185999999999933 ,0.22387200000000002, 0.787124],
   [0.452455,0.706881, 0.03773700000000002, 0.028317999999999954 ,0.22373400000000004, 0.786992],
   [0.452925,0.707163, 0.038207000000000046, 0.02859999999999996 ,0.22326400000000002, 0.78671],
   [0.440119,0.723553, 0.025401000000000007, 0.044989999999999974 ,0.23607000000000006, 0.77032],
   [0.441329,0.726223, 0.0012100000000000444, 0.00266999999999995 ,0.23486, 0.76765],
   [0.443896,0.739605, 0.0037770000000000303, 0.016051999999999955 ,0.23229300000000003, 0.754268],
   [0.425612,0.753974, -0.026843000000000006, 0.0446715836355942 ,0.25057700000000005, 0.739899],
   [0.445106,0.783061, 0.004987000000000019, 0.059508000000000005 ,0.23108300000000004, 0.710812],
   [0.424391,0.807819, -0.015727999999999964, 0.027599722849207864 ,0.251798, 0.686054],
   [0.459447,0.835801, 0.014340999999999993, 0.05274000000000001 ,0.21674200000000005, 0.658072],
   [0.482909,0.865377, 0.039012999999999964, 0.06140836688053186 ,0.19328000000000006, 0.628496],
   [0.510194,0.912967, 0.08458200000000005, 0.13422183006720356 ,0.165995, 0.580906],
   [0.516477,0.947884, 0.07137099999999996, 0.11569908042558896 ,0.15971200000000008, 0.5459890000000001],
   [0.540361,0.979489, 0.11596999999999996, 0.17843439297508923 ,0.13582800000000006, 0.514384],
   [0.584576,1.040285, 0.125129, 0.21250797967531287 ,0.09161300000000006, 0.4535880000000001],
   [0.581298,1.130299, 0.098389, 0.21556336113159036 ,0.09489100000000006, 0.36357400000000006],
   [0.575982,1.166143, 0.06578799999999996, 0.17426197263764176 ,0.10020700000000005, 0.3277300000000001],
   [0.570966,1.232599, 0.05448900000000001, 0.15977635867836337 ,0.10522300000000007, 0.261274],
   [0.567127,1.263118, 0.026766000000000068, 0.0654127027691036 ,0.10906199999999999, 0.23075500000000004],
   [0.559291,1.301374, -0.025285000000000002, 0.2505776888534603 ,0.11689800000000006, 0.19249899999999998],
   [0.585833,1.374949, 0.026542000000000066, 0.07357499999999995 ,0.09035599999999999, 0.11892400000000003],
   [0.593149,1.41069, 0.017167000000000043, 0.07228942987609699 ,0.08304, 0.083183],
     [0.592906,1.458012, 0.02194000000000007, 0.11534529616245104 ,0.083283, 0.03586099999999992],
     [0.626783,1.537817, 0.05965599999999993, 0.24137124735758952 ,0.30522499999999997, 0.121367],
     [0.625513,1.568912, 0.066222, 0.2541281085926119 ,0.30649499999999996, 0.09027199999999991],
     [0.629545,1.569823, 0.04371199999999997, 0.17480333731072495 ,0.3024629999999999, 0.08936100000000002],
     [0.635974,1.567877, 0.042825, 0.14537370862648835 ,0.2960339999999999, 0.09130700000000003],
     [0.635699,1.566926, 0.04279299999999997, 0.11270604778410598 ,0.29630899999999993, 0.09225799999999995],
     [0.637134,1.569763, 0.010350999999999999, 0.03194600000000003 ,0.29487399999999997, 0.08942099999999997],
     [0.63453,1.567649, 0.007747000000000059, 0.02983200000000008 ,0.2974779999999999, 0.09153499999999992],
     [0.641795,1.564561, 0.015012000000000025, 0.0267440000000001 ,0.29021299999999994, 0.0946229999999999],
     [0.635988,1.567554, 0.009205000000000019, 0.029736999999999902 ,0.29601999999999995, 0.0916300000000001],
     [0.636814,1.567218, 0.010031000000000012, 0.02940100000000001 ,0.29519399999999996, 0.09196599999999999],
     [0.6374,1.567879, 0.010616999999999988, 0.030062000000000033 ,0.294608, 0.09130499999999997],
     [0.632321,1.569623, 0.005538000000000043, 0.031806 ,0.2996869999999999, 0.089561],
     [0.635512,1.567917, 0.008728999999999987, 0.030100000000000016 ,0.296496, 0.09126699999999999],
     [0.638202,1.568094, 0.011419000000000068, 0.03027700000000011 ,0.2938059999999999, 0.0910899999999999],
     [0.630985,1.571082, 0.004202000000000039, 0.0332650000000001 ,0.30102299999999993, 0.0881019999999999],
     [0.639633,1.569333, 0.012850000000000028, 0.0315160000000001 ,0.29237499999999994, 0.0898509999999999],
     [0.636009,1.568086, 0.009226000000000067, 0.0302690000000001 ,0.2959989999999999, 0.0910979999999999],
     [0.641126,1.567251, 0.014342999999999995, 0.02943399999999996 ,0.290882, 0.09193300000000004],
     [0.646454,1.567846, 0.019670999999999994, 0.030029000000000083 ,0.285554, 0.09133799999999992],
     [0.647804,1.570379, 0.021021000000000067, 0.03256199999999998 ,0.2842039999999999, 0.08880500000000002],
     [0.636463,1.575283, 0.009680000000000022, 0.037466 ,0.29554499999999995, 0.083901],
     [0.646441,1.603597, 0.019658000000000064, 0.06577999999999995 ,0.2855669999999999, 0.05558700000000005],
     [0.652111,1.612951, 0.005669999999999953, 0.009354000000000084 ,0.27989699999999995, 0.04623299999999997],
     [0.659131,1.62792, 0.01267700000000005, 0.03192366242765998 ,0.2728769999999999, 0.03126399999999996],
     [0.679999,1.648899, 0.032194999999999974, 0.05790660545989179 ,0.2520089999999999, 0.0102850000000001],
     [0.706028,1.676706, 0.06956499999999999, 0.09470057987466829 ,0.22597999999999996, -0.017522000000000038],
     [0.710927,1.702251, 0.06448599999999993, 0.08748878037767138 ,0.22108099999999997, -0.043066999999999966],
     [0.742244,1.730182, 0.09013300000000002, 0.11674167831461624 ,0.18976399999999993, -0.07099800000000012],
     [0.75853,1.758804, 0.09939900000000002, 0.1310280038206746 ,0.1734779999999999, -0.09962000000000004],
     [0.785884,1.775132, 0.105885, 0.1303440672168994 ,0.14612399999999992, -0.11594799999999994],
     [0.802653,1.808356, 0.09662499999999996, 0.11805564080410047 ,0.129355, -0.14917200000000008],
     [0.815583,1.822441, 0.10465599999999997, 0.12023970865279732 ,0.116425, -0.16325699999999999],
     [0.830955,1.832433, 0.08871099999999998, 0.10274519882507405 ,0.10105299999999995, -0.173249],
     [0.859451,1.883668, 0.10092099999999993, 0.12540422465251977 ,0.07255699999999998, -0.2244839999999999],
     [0.877581,1.894914, 0.09169700000000003, 0.11936902603386601 ,0.05442699999999989, -0.23573],
     [0.891198,1.914456, 0.0885450000000001, 0.10876630001142451 ,0.0408099999999999, -0.25527199999999994],
     [0.905018,1.927864, 0.08943500000000004, 0.1098207055043523 ,0.02698999999999996, -0.26868000000000003],
     [0.90556,1.92715, 0.07460500000000003, 0.09158648859920104 ,0.026447999999999916, -0.2679659999999999],
     [0.904439,1.926309, 0.04498800000000003, 0.04490297454483594 ,0.027568999999999955, -0.26712500000000006],
     [0.905054,1.926788, 0.02747299999999997, 0.031104699424817417 ,0.026953999999999922, -0.26760399999999995],
     [0.905463,1.926779, 0.0004450000000000287, -0.0010850000000000026 ,0.02654499999999993, -0.267595],
     [0.905199,1.926852, 0.00018099999999998673, -0.0010120000000000129 ,0.026808999999999972, -0.267668],
     [0.905802,1.927112, 0.0007840000000000069, -0.0007520000000000859 ,0.02620599999999995, -0.26792799999999994],
     [0.918606,1.936422, 0.013588000000000044, 0.008558000000000066 ,0.013401999999999914, -0.2772380000000001],
     [0.946634,1.962975, 0.04157999999999995, 0.03611881685790941 ,-0.014626000000000028, -0.3037909999999999],
     [0.971773,2.01149, 0.06630999999999998, 0.07949472108400935 ,-0.03976500000000005, -0.3523059999999998],
     [0.998116,2.033524, 0.09291700000000003, 0.11094377161848556 ,-0.06610800000000006, -0.3743399999999999],
     [1.008653,2.052172, 0.10285100000000003, 0.12778312722015128 ,-0.07664500000000007, -0.3929880000000001],
     [1.020638,2.056923, 0.1020319999999999, 0.1270182250081513 ,-0.08862999999999999, -0.39773899999999984],
     [1.027176,2.062199, 0.08054200000000011, 0.0965289783375416 ,-0.09516800000000014, -0.4030150000000001],
     [1.025997,2.062964, 0.05422400000000005, 0.05111400000080124 ,-0.0939890000000001, -0.40378000000000003],
     [1.021999,2.059758, 0.0238830000000001, 0.022268889460220098 ,-0.08999100000000015, -0.400574],
     [1.027128,2.063176, 0.01847500000000002, 0.011003999999999792 ,-0.0951200000000001, -0.4039919999999999],
     [1.026316,2.063293, 0.017662999999999984, 0.011120999999999714 ,-0.09430800000000006, -0.40410899999999983],
     [1.021969,2.063051, 0.013315999999999883, 0.010879000000000083 ,-0.08996099999999996, -0.4038670000000002],
     [1.02141,2.060339, 0.012756999999999907, 0.008166999999999813 ,-0.08940199999999998, -0.40115499999999993],
     [1.021301,2.0658, 0.012647999999999993, 0.013627999999999751 ,-0.08929300000000007, -0.40661599999999987],
     [1.018289,2.07041, 0.009635999999999978, 0.018237999999999754 ,-0.08628100000000005, -0.41122599999999987],
     [1.018517,2.072358, 0.009863999999999873, 0.020185999999999815 ,-0.08650899999999995, -0.41317399999999993],
     [1.018854,2.068471, 0.010200999999999905, 0.016299000000000063 ,-0.08684599999999998, -0.4092870000000002],
     [1.021423,2.070834, 0.012769999999999948, 0.018661999999999956 ,-0.08941500000000002, -0.41165000000000007],
     [1.019767,2.070211, 0.011114000000000068, 0.018038999999999916 ,-0.08775900000000014, -0.41102700000000003],
     [1.02015,2.070468, 0.011496999999999868, 0.018295999999999868 ,-0.08814199999999994, -0.411284],
     [1.026239,2.069051, 0.01758599999999988, 0.016878999999999866 ,-0.09423099999999995, -0.409867],
     [1.034357,2.07349, 0.02570399999999995, 0.021317999999999948 ,-0.10234900000000002, -0.41430600000000006],
     [1.044862,2.069952, 0.036208999999999936, 0.017779999999999685 ,-0.11285400000000001, -0.4107679999999998],
     [1.061227,2.081622, 0.04145999999999983, 0.009745746563375679 ,-0.12921899999999997, -0.42243799999999987],
     [1.093416,2.113829, 0.07326600000000005, 0.04348550822191477 ,-0.161408, -0.45464499999999997],
     [1.124985,2.14266, 0.098746, 0.07610918657029275 ,-0.19297699999999995, -0.4834759999999998],
     [1.154562,2.158099, 0.12020500000000012, 0.09433238972804503 ,-0.22255400000000014, -0.498915],
     [1.179774,2.177101, 0.13491200000000014, 0.10870173143267348 ,-0.24776600000000015, -0.517917],
     [1.205114,2.195326, 0.1438870000000001, 0.11086167418547621 ,-0.27310600000000007, -0.5361420000000001],
     [1.239154,2.21326, 0.14573800000000015, 0.09876480991685188 ,-0.30714600000000014, -0.554076],
     [1.276566,2.233174, 0.1515810000000002, 0.0925541714168503 ,-0.34455800000000014, -0.57399],
     [1.30721,2.244353, 0.1526479999999999, 0.086054085646325 ,-0.37520200000000004, -0.5851689999999998],
     [1.327038,2.248269, 0.14726399999999984, 0.07166676450648746 ,-0.39503, -0.5890850000000001],
     [1.337658,2.248542, 0.132544, 0.054651064808993066 ,-0.40565000000000007, -0.589358],
     [1.35075,2.265929, 0.1115959999999998, 0.04565406703907149 ,-0.41874199999999995, -0.6067449999999999],
     [1.369389,2.270181, 0.09282299999999988, 0.037232851905811604 ,-0.437381, -0.610997],
     [1.398272,2.279951, 0.09106199999999998, 0.039227804198979754 ,-0.466264, -0.6207670000000001]
])

v1 = Vector(Point(0,0), Point(-0.000476, 0.00028907))
v2 = Vector(Point(0,0), Point(0.079737, -0.130236))
print(v1.clockwise_angle_between(v2))
print(v1.clockwise_angle_between(v2) > np.pi)

p1 = Point(0.091812, 0.03647197234736366)
p2 = Point(-0.00042799999999998395, -0.000600009779178956)

print(Point(0, 0).distance(p1))
print(Point(0, 0).distance(p2))

x, y, hx, hy, rx, ry = real_set.T
plt.quiver(x, y, hx, hy, color='b')
plt.quiver(x, y, rx, ry, color='r')
plt.pause(0.05)

# plt.plot(0, 0, 'bo')
# plt.plot(v1[0], v1[1], 'ro')
# plt.plot(v2[0], v2[1], 'ro')
# plt.plot(0.002772, 0.00128472, 'bo')
# plt.plot(0.078554, 0.102219, 'bo')
# plt.pause(0.05)

input("Press Enter to continue...")
