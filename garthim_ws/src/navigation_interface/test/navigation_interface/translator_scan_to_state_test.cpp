#include <gtest/gtest.h>
#include <navigation_interface/translator_to_state.h>

namespace navigation_interface {
    class TranslatorScanToStateTest : public testing::Test {
        public:
        unsigned seed = 0;

        sensor_msgs::LaserScan ls;
        TranslatorScanToStateTest() {
            ls.header.frame_id = "scan";
            ls.header.stamp = ros::Time::now();
            ls.angle_increment = 0.00163668883033;
            ls.angle_min = -0.521567881107;
            ls.angle_max =  0.524276316166;
            ls.angle_increment = 0.00163668883033;
            ls.time_increment =  0.0;
            ls.scan_time = 0.0329999998212;
            ls.range_min = 0.449999988079;
            ls.range_max = 10.0;
            ls.ranges = {2.2569990158081055, 2.2557265758514404, 2.254457473754883, 2.25319242477417, 2.2506728172302246, 2.2494184970855713, 2.2481679916381836, 2.2469210624694824, 2.245678186416626, 2.244438648223877, 2.241971015930176, 2.2407431602478027, 2.239518642425537, 2.238298177719116, 2.237081289291382, 2.234659194946289, 2.2334539890289307, 2.232252597808838, 2.2310550212860107, 2.229861259460449, 2.2286715507507324, 2.226303815841675, 2.225125551223755, 2.223951578140259, 2.2227814197540283, 2.2216150760650635, 2.2204530239105225, 2.219294786453247, 2.2169904708862305, 2.21584415435791, 2.2147018909454346, 2.213564157485962, 2.2124297618865967, 2.2112998962402344, 2.210174083709717, 2.207934617996216, 2.2068212032318115, 2.205711841583252, 2.204606533050537, 2.203505516052246, 2.2024085521698, 2.2013158798217773, 2.2002272605895996, 2.1980631351470947, 2.1969873905181885, 2.195915937423706, 2.1948485374450684, 2.1937854290008545, 2.1927268505096436, 2.1916723251342773, 2.190622329711914, 2.1895766258239746, 2.187498092651367, 2.1864655017852783, 2.1854372024536133, 2.184413194656372, 2.183393955230713, 2.1823787689208984, 2.181368112564087, 2.1803619861602783, 2.1793601512908936, 2.1783628463745117, 2.176381826400757, 2.175398111343384, 2.1744186878204346, 2.1734440326690674, 2.172473907470703, 2.171508312225342, 2.1705472469329834, 2.169590950012207, 2.1686389446258545, 2.167691946029663, 2.1667492389678955, 2.16581130027771, 2.1648778915405273, 2.163025379180908, 2.1621060371398926, 2.161191463470459, 2.1602816581726074, 2.159376382827759, 2.1584761142730713, 2.1575803756713867, 2.1566896438598633, 2.1558034420013428, 2.1549222469329834, 2.154045581817627, 2.1531741619110107, 2.1523072719573975, 2.151445150375366, 2.150588274002075, 2.149735927581787, 2.14888858795166, 2.1480460166931152, 2.1472086906433105, 2.145548105239868, 2.1447253227233887, 2.1439075469970703, 2.143094778060913, 2.142286777496338, 2.141484260559082,
             2.140686511993408, 2.1398935317993164, 2.139105796813965, 2.1383230686187744, 2.137545347213745, 2.136772871017456, 2.136005401611328, 2.1352429389953613, 2.1344857215881348, 2.1337335109710693, 2.132986545562744, 2.132244825363159, 2.1315081119537354, 2.1307766437530518, 2.1300501823425293, 2.129329204559326, 2.1286134719848633, 2.1279027462005615, 2.127197265625, 2.1264970302581787, 2.1258022785186768, 2.125112533569336, 2.1244282722473145, 2.123749017715454, 2.123075485229492, 2.1224071979522705, 2.121744155883789, 2.121086597442627, 2.120434284210205, 2.1197872161865234, 2.1191458702087402, 2.118509531021118, 2.1178789138793945, 2.117253541946411, 2.116633892059326, 2.1160192489624023, 2.115410566329956, 2.11480712890625, 2.1142091751098633, 2.113616704940796, 2.1130294799804688, 2.112448215484619, 2.111872434616089, 2.111302375793457, 2.1107375621795654, 2.110178232192993, 2.1096246242523193, 2.109076976776123, 2.108534574508667, 2.1079976558685303, 2.107466697692871, 2.1069412231445312, 2.10642147064209, 2.105907440185547, 2.1053988933563232, 2.104896306991577, 2.1043994426727295, 2.103907823562622, 2.1034224033355713, 2.10294246673584, 2.102468729019165, 2.1020004749298096, 2.1015379428863525, 2.101081132888794, 2.100630521774292, 2.1001851558685303, 2.0997462272644043, NAN, 2.0993127822875977, 2.0988852977752686, 2.098463535308838, 2.0980474948883057, 2.09763765335083, 2.097234010696411, 2.0968356132507324, 2.0964434146881104, 2.096057176589966, 2.095676898956299, 2.0953025817871094, 2.0949339866638184, 2.094571828842163, 2.0942153930664062, 2.093864917755127, 2.0935206413269043, 2.093182325363159, 2.0928499698638916, 2.0925235748291016, 2.092203378677368, 2.0918893814086914, 2.091581344604492, NAN, 2.0912790298461914, 2.0909831523895264, 2.090693473815918, 2.090409755706787, 2.090132236480713, 2.0898609161376953, 2.0895957946777344, 2.089336633682251, 2.089083671569824, 2.088837146759033, 2.0885965824127197,
             2.088362216949463, 2.088134288787842, 2.0879123210906982, 2.0876967906951904, 2.0874874591827393, NAN, 2.087284564971924, 2.087087631225586, 2.086897134780884, 2.0867128372192383, 2.0865349769592285, 2.0863633155822754, 2.086198091506958, 2.086038827896118, 2.085886240005493, 2.085740089416504, 2.0856001377105713, 2.0854666233062744, 2.085339307785034, 2.085218667984009, NAN, 2.08510422706604, 2.084996223449707, 2.084894895553589, 2.0847995281219482, 2.0847110748291016, 2.0846285820007324, 2.0845532417297363, 2.0844836235046387, 2.084420919418335, 2.084364652633667, 2.0843148231506348, 2.0842714309692383, 2.0842347145080566, NAN, 2.0842044353485107, 2.0841808319091797, 2.0841636657714844, 2.0841526985168457, 2.084148645401001, 2.084151268005807159424, 2.0994207859039307, 2.098989963531494, 2.0985658168792725, 2.0981478691101074, 2.0977368354797363, 2.097332000732422, 2.0969338417053223, 2.0965418815612793, 2.096156597137451, 2.095777988433838, 2.0954055786132812, NAN, 2.0950398445129395, 2.0946807861328125, 2.0943281650543213, 2.093981981277466, 2.093642473220825, 2.0933094024658203, 2.0929830074310303, 2.092663049697876, 2.0923500061035156, 2.092043161392212, 2.091742992401123, 2.091449499130249, NAN, 2.0911624431610107, 2.0908823013305664, 2.0906083583831787, 2.090341091156006, 2.090080976486206, 2.089826822280884, 2.0895795822143555, 2.089339256286621, 2.0891048908233643, 2.0888776779174805, 2.0886569023132324, NAN, 2.0884430408477783, 2.08823561668396, 2.0880346298217773, 2.0878405570983887, 2.087653398513794, 2.087472677230835, 2.0872983932495117, 2.0871307849884033, 2.086970329284668, 2.0868165493011475, NAN, 2.0866689682006836, 2.0865285396575928, 2.0863945484161377, 2.0862674713134766, 2.0861470699310303, 2.0860331058502197, 2.085926055908203, 2.0858256816864014, 2.0857319831848145, 2.0856451988220215, 2.0855648517608643, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 2.0855109691619873, 2.0855863094329834, 2.0856685638427734, 2.0857579708099365, 2.0858538150787354, 2.085956335067749, 2.0860657691955566, 2.0861823558807373, NAN, 2.0863051414489746, 2.086435317993164, 2.0865719318389893, 2.0867154598236084, 2.0868659019470215, 2.0870230197906494, 2.0871870517730713, 2.087357759475708, 2.0875351428985596, 2.0877199172973633, NAN, 2.0879108905792236, 2.088108777999878, 2.0883138179779053, 2.0885255336761475, 2.0887439250946045, 2.0889694690704346, 2.0892014503479004, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 2.0956528186798096, 2.0960350036621094, 2.0964243412017822, NAN, 2.0968198776245117, 2.0972228050231934, 2.09763240814209, 2.098048448562622, 2.0984716415405273, 2.0989017486572266, 2.0993382930755615, 2.0997817516326904, 2.1002321243286133, 2.100689172744751, NAN, 2.1011531352996826, 2.10162353515625, 2.1021013259887695, 2.102585792541504, 2.103076457977295, 2.103574514389038, 2.104079484939575, 2.104590654373169, 2.1051089763641357, 2.1056342124938965, 2.106165647506714, NAN, 2.1067044734954834, 2.1072497367858887, 2.107801914215088, 2.108361005783081, 2.108926296234131, 2.109498977661133, 2.1100783348083496, 2.110664129257202, 2.1112565994262695, 2.11185622215271, 2.112462282180786, 2.1130752563476562, NAN, 2.1136951446533203, 2.11432147026062, 2.114954710006714, 2.1155946254730225, 2.116241216659546, 2.1168947219848633, 2.1175544261932373, 2.1182212829589844, 2.1188950538635254, 2.119575023651123, 2.1202619075775146, 2.1209557056427, NAN, 2.1216559410095215, 2.1223630905151367, 2.123076915740967, 2.1237969398498535, 2.1245243549346924, 2.125258207321167, 2.1259984970092773, 2.1267454624176025, 2.1274993419647217, 2.1282598972320557, 2.1290271282196045, 2.12980055809021, 2.1305809020996094, 2.131368398666382, NAN, 2.132161855697632, 2.132962226867676, 2.1337690353393555, 2.134582757949829, 2.1354029178619385, 2.136229991912842, 2.1370630264282227, 2.1379032135009766, 2.1387500762939453, 2.1396028995513916, 2.140462875366211, 2.141329050064087, 2.142202138900757, 2.1430816650390625, 2.143967866897583, NAN, 2.1448607444763184, 2.1457598209381104, 2.1466658115386963, 2.147578001022339, 2.1484971046447754, 2.1494224071502686, 2.1503543853759766, 2.1512930393218994, 2.152237892150879, 2.1531894207000732, 2.1541476249694824, 2.155111789703369, 2.156083106994629, 2.1570606231689453, 2.1580445766448975, 2.1590352058410645, 2.160032033920288, 2.1610352993011475, 2.1620452404022217, NAN, 2.1630616188049316, 2.1640844345092773, 2.165113687515259, 2.166149377822876, 2.16719126701355, 2.1682398319244385, 2.169294595718384, 2.170356035232544, 2.1714236736297607, 2.172497510910034, 2.1735782623291016, 2.1746654510498047, 2.175758123397827, 2.1768579483032227, 2.177963972091675, 2.1790761947631836, 2.1801950931549072, 2.1813199520111084, 2.1824512481689453, 2.183588981628418, 2.1847329139709473, 2.185883045196533, 2.187039613723755, 2.1882028579711914, 2.1893715858459473, 2.190547227859497, 2.1917290687561035, 2.1929168701171875, 2.1941113471984863, 2.1953117847442627, 2.1965184211730957, 2.1977314949035645, 2.1989505290985107, 2.2001757621765137, 2.2014076709747314, 2.2026455402374268, 2.2038893699645996, 2.205139636993408, NAN, 2.2063961029052734, 2.207658529281616, 2.2089273929595947, 2.210202217102051, 2.2114830017089844, 2.2127702236175537, 2.214063882827759, 2.215363025665283, 2.2166686058044434, 2.2179806232452393, 2.2192981243133545, 2.2206220626831055, 2.221951961517334, 2.223288059234619, 2.224630117416382, 2.225978374481201, 2.227332353591919, 2.2300591468811035, 2.231431245803833, 2.232809543609619, 2.234194040298462, 2.235584259033203, 2.236980676651001, 2.2383830547332764, 2.23979115486145, 2.2412054538726807, 2.2426257133483887, 2.244051933288574, 2.2454841136932373, 2.246922254562378, 2.248365879058838, 2.2498159408569336, 2.2512717247009277, 2.252732992172241, 2.2542009353637695, 2.255674362182617, 2.2571535110473633, 2.258638858795166, 2.260129928588867, 2.261626720428467, 2.263129472732544, 2.2646381855010986,
             2.2661523818969727, 2.267672538757324, 2.269198417663574, 2.2707302570343018, 2.2722678184509277, 2.273811101913452, 2.275360107421875, 2.2769150733947754, 2.278475761413574, 2.281614065170288, 2.283191680908203, 2.2847752571105957, 2.2863643169403076, 2.287959337234497, 2.289560079574585, 2.291166067123413, 2.2927777767181396, 2.294395685195923, 2.296018600463867, 2.297647476196289, 2.2992820739746094, 2.300921678543091, 2.302567481994629, 2.3042190074920654, 2.305875539779663, 2.309206247329712, 2.310879707336426, 2.312558889389038, 2.3142435550689697, 2.3159337043762207, 2.317629337310791, 2.3193306922912598, 2.3210372924804688, 2.322749376296997, 2.324467182159424, 2.32619047164917, 2.327918767929077, 2.331392526626587, 2.3331375122070312, 2.334887981414795, 2.336643695831299, 2.338404893875122, 2.3401715755462646, 2.3419437408447266, 2.3437209129333496, 2.34550404548645, 2.347292184829712, 2.350884437561035, 2.3526885509490967, 2.3544979095458984, 2.3563125133514404, 2.3581326007843018, 2.3599581718444824, 2.361788511276245, 2.3636248111724854, 2.36731219291687, 2.369163990020752, 2.371021032333374, 2.3728830814361572, 2.3747506141662598, 2.3766231536865234, 2.3785009384155273, 2.3803839683532715, 2.3841652870178223, 2.386063814163208, 2.387967586517334, 2.389876365661621, 2.3917899131774902, 2.3937089443206787, 2.3956332206726074, 2.399496555328369, 2.4014360904693604, 2.4033803939819336, 2.405329942703247, 2.4072844982147217, 2.4092440605163574, 2.4131784439086914, 2.4151532649993896, 2.417132616043091, 2.4191174507141113, 2.421107292175293, 2.4231016635894775, 2.4271059036254883, 2.429115056991577, 2.4311296939849854, 2.4331490993499756, 2.435173273086548, 2.439236640930176, 2.4412755966186523, 2.443319320678711, 2.4453678131103516, 2.4474215507507324, 2.449479579925537, 2.453611135482788, 2.455683708190918, 2.457761764526367, 2.4598441123962402, 2.4619317054748535};
        }
        virtual void TestBody(){}
        
    };
    TEST(TranslatorScanToStateTest, scanToState) {
        TranslatorScanToStateTest t;
        TranslatorToState ts;
        CollisionStatus s;
        //ts.getObstaclesState(t.ls,s);
        ASSERT_EQ(0, 0);
    }
}