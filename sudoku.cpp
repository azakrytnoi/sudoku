/*
 * sudoky.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: n69136
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <array>
#include <algorithm>
#include <map>
#include <cstdint>
#include <iterator>
#include <memory>
#include <future>

#include <random>

class board {
public:
    typedef uint16_t cell_t;
    typedef std::array<std::array<cell_t, 9>, 9> board_t;

private:
    std::array<std::array<cell_t, 9>, 9> board_;

public:
    board() :
            board_(std::array<std::array<cell_t, 9>, 9>()) {
    }

    board_t& operator ()() {
        return board_;
    }

    friend std::ostream& operator <<(std::ostream& out, const board& b) {
        std::ostream_iterator<cell_t> oi(out, " ");
        std::for_each(b.board_.begin(), b.board_.end(), [&](const std::array<cell_t, 9>& row) {
            std::copy(row.begin(), row.end(), oi);
            out << std::endl;
        });
        return out;
    }

    friend std::istream& operator >>(std::istream& in, board& b) {
        std::istream_iterator<cell_t> iin(in);
        std::for_each(b.board_.begin(), b.board_.end(), [&](std::array<cell_t, 9>& row) {
            std::generate(row.begin(), row.end(), [&]() {return *(iin++);});
        });
        return in;
    }
};

namespace recursive {

    class solver {
        size_t max_depth_;
    public:
        solver() :
                max_depth_(0) {
        }

        bool operator()(board& b) {
            max_depth_ = 0;
            return solve(b());
        }

        size_t max_depth() const {
            return max_depth_;
        }

        inline
        bool is_valid(board::board_t& work, size_t row, size_t col);

    private:
        bool solve(board::board_t& work);
    };

    bool solver::solve(board::board_t& work) {
        max_depth_++;
        for (size_t row = 0; row < 9; row++) {
            for (size_t col = 0; col < 9; col++) {
                if (work[row][col] == 0) {
                    for (board::cell_t k = 1; k <= 9; k++) {
                        work[row][col] = k;
                        if (is_valid(work, row, col) && solve(work)) {
                            return true;
                        }
                        work[row][col] = 0;
                    }
                    max_depth_--;
                    return false;
                }
            }
        }
        return true;
    }

    bool solver::is_valid(board::board_t& work, size_t row, size_t col) {
        auto check_fn = [](std::array<bool,10>& check, board::cell_t val) {
            if (val != 0) {
                auto tmp = check[val];
                check[val] = true;
                return tmp;
            }
            return false;
        };

        std::future<bool> check_row(std::async(std::launch::deferred, [check_fn, work, row, col]() {
            std::array<bool,10> check;
            std::fill(check.begin(), check.end(), false);
            for (size_t idx=0; idx < 9; idx++) {
                if (check_fn(check, work[row][idx])) {
                    return false;
                }
            }
            return true;
        }));

        std::future<bool> check_col(std::async(std::launch::deferred, [check_fn, work, row, col]() {
            std::array<bool,10> check;
            std::fill(check.begin(), check.end(), false);
            for (size_t idx = 0; idx < 9; idx++) {
                if (work[idx][col] != 0) {
                    if (check_fn(check, work[idx][col])) {
                        return false;
                    }
                }
            }
            return true;
        }));

        std::future<bool> check_sq(std::async(std::launch::deferred, [check_fn, work, row, col]() {
            std::array<bool,10> check;
            std::fill(check.begin(), check.end(), false);
            for (size_t drx = 0; drx < 3; drx++) {
                for (size_t dcx = 0; dcx < 3; dcx++) {
                    size_t rx = row / 3 * 3 + drx;
                    size_t cx = col / 3 * 3 + dcx;
                    if (work[rx][cx] != 0) {
                        if (check_fn(check, work[rx][cx])) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }));

        return check_row.get() && check_col.get() && check_sq.get();
    }

}  // namespace recursive

void sudoku() {
//    std::ifstream in("sample.txt");
//    if (in) {
//        board b;
//        in >> b;
//        std::cout << b << "====================" << std::endl;
//        recursive::solver s;
//        if (s(b)) {
//            std::cout << s.max_depth() << std::endl;
//            std::cout << b << std::endl;
//        } else {
//            std::cout << "No solution" << std::endl << b;
//        }
//    }
//
    std::random_device rd;
    std::mt19937_64 mersenne(rd());
    std::uniform_int_distribution<size_t> distr(0, 8);
//    std::default_random_engine rgen;

    auto gen = std::bind(distr, mersenne);
    std::vector<board::cell_t> init( { 1, 2, 3, 4, 5, 6, 7, 8, 9 });
    board b;
    recursive::solver s;
    for (uint32_t cnt = 0; cnt < 4; cnt++) {
        std::shuffle(init.begin(), init.end(), rd);
        std::for_each(init.begin(), init.end(), [&](board::cell_t cell) {
            if (gen() % 3 <= 0) {
                while(true) {
                    size_t row (gen());
                    size_t col(gen());
                    if (b()[row][col] == 0) {
                        b()[row][col] = cell;
                        if (s.is_valid(b(), row, col)) {
                            break;
                        } else {
                            b()[row][col] = 0;
                        }
                    }
                }
            }

        });
    }
    std::cout << b << "====================" << std::endl;
    if (s(b)) {
        std::cout << s.max_depth() << std::endl;
        std::cout << b << std::endl;
    } else {
        std::cout << "No solution" << std::endl << b;
    }
}
