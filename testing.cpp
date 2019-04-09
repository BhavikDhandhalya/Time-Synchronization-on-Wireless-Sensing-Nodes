#include "iostream"
#include "map"
#include "queue"
using namespace std;

#define LL long long

const int NN = 500;

int N;
int node_id;
int counter;
int TOTAL_ITER;
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

/*CLUSTER MEMMBER*/
double t3;
double m_vc[NN];
double v_clk_member[NN];
double skew_cmp_member[NN];
double off_cmp_member[NN];
double prev_off_cmp_member[NN];
double prev_skew_cmp_member[NN];
double avg_m_relative_skew_cmp[NN];

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
	freopen("out_close.txt", "r", stdin);
	freopen("send/clean_out_close.txt", "w", stdout);

	int prev_counter = 0, cnt_iteration = 0;
	queue < pair < LL, pair < LL, LL > > > Q;
	queue < pair < LL, LL > > Q2;

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
		Q2.push(make_pair(t3, recTS_of_head));

		if (hash_counter[counter] == 4) {
			TOTAL_ITER++;
			for (int ab = 0; ab < 4; ab++) {
				while (Q.size() > 4) {
					Q.pop();
					Q2.pop();
				}
				auto X = Q.front();
				Q.pop();

				auto Y = Q2.front();
				Q2.pop();

				array_sendTS[TOTAL_ITER] = X.first;
				child_recvTS[X.second.first][TOTAL_ITER] = X.second.second;

				cout << TOTAL_ITER << " ";
				cout << " node_id = " << X.second.first << " t1 " << X.first << " " << " t2 " << X.second.second;
				cout << " t3 = " << Y.first;
				cout << " t4 = " << Y.second;
				cout << endl;
			}

			while (!Q.empty()) {
				Q.pop();
				Q2.pop();
			}
		}
	}

	cout << TOTAL_ITER << endl;
	cout << "Testing ..." << endl;

	memset(hash_counter, 0, sizeof(hash_counter));

	TOTAL_ITER = 0;
	itr_counter = 0;
	while (cin >> temp) {
		string ign;

		if (temp == "test") break;

		/*1553078685435: 2	2	sendTS = 845980	recTS = 2364899868	sendTS_of_member = 2364909822	receiveTS_of_head = 868223*/
		cin >> node_id >> counter >> ign >> ign >> sendTS >> ign >> ign >> recvTS;
		cin >> ign >> ign >> t3 >> ign >> ign >> recTS_of_head;
		hash_counter[counter]++;

		//string a1, a2, a3; cin >> a1 >> a2 >> a3;
		Q.push(make_pair(sendTS, make_pair(node_id, recvTS)));
		Q2.push(make_pair(t3, recTS_of_head));

		if (hash_counter[counter] == 5) {
			TOTAL_ITER++;
			for (int ab = 0; ab < 5; ab++) {
				while (Q.size() > 5) {
					Q.pop();
					Q2.pop();
				}
				auto X = Q.front();
				Q.pop();

				auto Y = Q2.front();
				Q2.pop();

				array_sendTS[TOTAL_ITER] = X.first;
				child_recvTS[X.second.first][TOTAL_ITER] = X.second.second;

				cout << TOTAL_ITER << " ";
				cout << " node_id = " << X.second.first << " t1 = " << X.first << " " << " t2 = " << X.second.second;
				cout << " t3 = " << Y.first;
				cout << " t4 = " << Y.second;
				cout << endl;

			}

			while (!Q.empty()) {
				Q.pop();
				Q2.pop();
			}
		}
	}

	cout << TOTAL_ITER << endl;

	return 0;
}