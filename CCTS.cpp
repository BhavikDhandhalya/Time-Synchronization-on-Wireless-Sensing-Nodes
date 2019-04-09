#include "iostream"
#include "map"
#include "queue"
#include <math.h>
using namespace std;

#define LL long long

const int NN = 500;

int N = 4;
int node_id;
int counter;
int TOTAL_ITER;
int total_iterations;
long long sendTS;
long long recvTS;
long long hash_counter[NN];
long long array_sendTS[NN];
long long child_recvTS[NN][NN];

map < int, int > foff;
map < int, int > ::iterator it;

/*CLUSTER HEAD*/
double h_vc;
double roww;
double avg_v_clk;
double h_avg_skew_cmp;
double h_skew_cmp;
double h_prev_skew_cmp;
double h_final_rel;
double h_offset_cmp;
double h_prev_offset_cmp;
double prev_v_clk;
double h_rel_skew[NN];
double recTS_of_head;

double h_estimated_skew;
double h_estimated_offset;

/*CLUSTER MEMMBER*/
double t3;
double m_vc[NN];
double v_clk_member[NN];
double skew_cmp_member[NN];
double off_cmp_member[NN];
double prev_off_cmp_member[NN];
double prev_skew_cmp_member[NN];
double avg_m_relative_skew_cmp[NN];

double m_estimated_skew[NN];
double m_estimated_offset[NN];

void init() {
	h_offset_cmp = 0.0;
	h_skew_cmp = 1.0;
	h_prev_skew_cmp = 1.0;
	h_prev_offset_cmp = 0.0;

	for (int i = 1; i <= N; i++) {
		skew_cmp_member[i] = 1.0;
		off_cmp_member[i] = 0.0;
		prev_skew_cmp_member[i] = 1.0;
		prev_off_cmp_member[i] = 0.0;
	}
}

double find_avg_v_clk(int i) {
	double result = 0.0;
	for (int j = 1; j <= N; j++)
		result += v_clk_member[j];

	return result / (1.0 * N);
}

int main() {
	freopen("ground_out_2.txt", "r", stdin);
	freopen("out/CCTS_ground_out_2.txt", "w", stdout);

	int prev_counter = 0, cnt_iteration = 0;
	queue < pair < LL, pair < LL, LL > > > Q;

	string temp;
	int itr_counter = 0;
	while (cin >> temp) {
		string ign;

		if (temp == "test") break;

		/*1553078685435: 2	2	sendTS = 845980	recTS = 2364899868	sendTS_of_member = 2364909822	receiveTS_of_head = 868223*/
		cin >> node_id >> counter >> ign >> ign >> sendTS >> ign >> ign >> recvTS;
		cin >> ign >> ign >> t3 >> ign >> ign >> recTS_of_head;
		hash_counter[counter]++;

		//string a1, a2, a3; cin >> a1 >> a2 >> a3;
		Q.push(make_pair(sendTS, make_pair(node_id, recvTS)));

		if (hash_counter[counter] == 4) {
			TOTAL_ITER++;
			for (int ab = 0; ab < 4; ab++) {
				while (Q.size() > 4) {
					Q.pop();
				}
				auto X = Q.front();
				Q.pop();

				array_sendTS[TOTAL_ITER] = X.first;
				child_recvTS[X.second.first][TOTAL_ITER] = X.second.second;
			}

			while (!Q.empty()) {
				Q.pop();
			}
		}
	}

	total_iterations = TOTAL_ITER;

	init();

	// 2nd iteration
	for (int k = 1; k <= N; k++)
		v_clk_member[k] = child_recvTS[k][2];

	h_vc = array_sendTS[2];

	// 3rd iteration
	for (int i = 3; i <= total_iterations; i++) {
		cout << i << " ";
		/*HEAD calculations*/
		/*eq. 9*/
		avg_v_clk = find_avg_v_clk(i);
		roww = (N * 1.0)/(double)(N + 1);

		/*eq. 18*/
		h_offset_cmp = h_prev_offset_cmp + (roww)*((float)((avg_v_clk - h_vc)));

		for (int k = 1; k <= N; k++)
			h_rel_skew[k] = (double)(child_recvTS[k][i-1] - child_recvTS[k][i-2])/(float)(array_sendTS[i-1] - array_sendTS[i-2]);

		h_final_rel = 0.0;
		for (int k = 1; k <= N; k++) 
			h_final_rel += skew_cmp_member[k] * h_rel_skew[k];

		h_final_rel /= (1.0 * N);

		/*eq 12. next two lines*/
		h_skew_cmp = (1.0 - roww) * h_prev_skew_cmp;
		h_skew_cmp = h_skew_cmp + (double)roww * h_final_rel;

		/*FOR TESTING*/
		h_estimated_skew = h_skew_cmp;
		h_estimated_offset = h_offset_cmp;

		/*eq. 7.*/
		h_vc = (double)h_prev_skew_cmp * array_sendTS[i-1] + (double)h_prev_offset_cmp;

		/*MEMBER calculations*/
		for (int k = 1; k <= N; k++) {
			/*eq. 19*/
			off_cmp_member[k] = prev_off_cmp_member[k] + (0.5)*(h_vc - v_clk_member[k]);

			if (child_recvTS[k][i] - child_recvTS[k][i-1] != 0)
				avg_m_relative_skew_cmp[k] = (double)(array_sendTS[i-1] - array_sendTS[i-2])/(double)(child_recvTS[k][i-1] - child_recvTS[k][i-2]);

			/*eq. 13.*/
			skew_cmp_member[k] = (0.5) * (double)prev_skew_cmp_member[k];
			skew_cmp_member[k] = skew_cmp_member[k] + (double)(0.5)*(h_prev_skew_cmp)*((double)avg_m_relative_skew_cmp[k]);


			/*FOR TESTING*/
			m_estimated_skew[k] = skew_cmp_member[k];
			m_estimated_offset[k] = off_cmp_member[k];
			m_vc[k] = (double)skew_cmp_member[k] * (double)child_recvTS[k][i] + (double)off_cmp_member[k];
			cout << " " << k << " = " << m_vc[k] << " ";
			//cout << endl;
			//cout << "skew = " << skew_cmp_member[k] << " offset = " << off_cmp_member[k] << endl; 

			v_clk_member[k] = m_vc[k];
			prev_off_cmp_member[k] = off_cmp_member[k];
			prev_skew_cmp_member[k] = skew_cmp_member[k];
		}

		h_vc = (double)h_skew_cmp * array_sendTS[i] + (double)h_offset_cmp;

		cout << "h_vc = " << h_vc << " ";

		h_prev_skew_cmp = h_skew_cmp;
		h_prev_offset_cmp = h_offset_cmp;

		cout << endl;
	}

	cout << TOTAL_ITER << endl << endl;

	//cout << int(abs(h_vc - m_vc[3])) << endl;

	double ERROR = 0.0;

	cout << "h_estimated_skew = " << h_estimated_skew << "  ";
	cout << "h_estimated_offset = " << h_estimated_offset << endl;
	for (int qq = 1; qq <= N; qq++) {
		cout << "m_id = " << qq << " ";
		cout << "m_estimated_skew = " << m_estimated_skew[qq] << " ";
		cout << "m_estimated_offset = " << m_estimated_offset[qq] << endl;
	}

	cout << "Testing ..." << endl << endl;
	memset(hash_counter, 0, sizeof(hash_counter));

	TOTAL_ITER = 0;
	itr_counter = 0;
	double h_recTS = 0.0;
	while (cin >> temp) {
		string ign;

		if (temp == "test") break;

		/*1553078685435: 2	2	sendTS = 845980	recTS = 2364899868	sendTS_of_member = 2364909822	receiveTS_of_head = 868223*/
		cin >> node_id >> counter >> ign >> ign >> sendTS >> ign >> ign >> recvTS;
		cin >> ign >> ign >> t3 >> ign >> ign >> recTS_of_head;
		hash_counter[counter]++;

		//string a1, a2, a3; cin >> a1 >> a2 >> a3;
		Q.push(make_pair(recTS_of_head, make_pair(node_id, recvTS)));

		if (node_id == 35) {
			h_recTS = recTS_of_head;
		}

		ERROR = 0.0;

		if (hash_counter[counter] == 5) {
			TOTAL_ITER++;
			cout << "Iteration = " << TOTAL_ITER << " ";
			for (int ab = 0; ab < 5; ab++) {
				while (Q.size() > 5) {
					Q.pop();
				}
				auto X = Q.front();
				Q.pop();

				array_sendTS[TOTAL_ITER] = X.first;
				child_recvTS[X.second.first][TOTAL_ITER] = X.second.second;

				ERROR += abs(
					(h_estimated_skew * (double)h_recTS + h_estimated_offset)
				-	(m_estimated_skew[X.second.first] * (double)X.second.second + m_estimated_offset[X.second.first])
					);

				//cout << ERROR << endl;

				//cout << TOTAL_ITER << " ";
				//cout << " node_id = " << X.second.first << " " << " recTS " << X.second.second;
				//cout << " sendTS " << X.first;
				//cout << endl;

			}

			while (!Q.empty()) {
				Q.pop();
			}

			ERROR /= 4.0;

			cout << "ERROR = " << ERROR << endl;
		} 
	}

	cout << TOTAL_ITER << endl;


	return 0;
}