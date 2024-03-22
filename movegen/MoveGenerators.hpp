#ifndef _FILO2_MOVEGENERATORS_HPP_
#define _FILO2_MOVEGENERATORS_HPP_

#include <functional>
#include <random>
#include <set>
#include <unordered_set>

#include "../base/BinaryHeap.hpp"
#include "../base/Flat2DVector.hpp"
#include "../base/SparseIntSet.hpp"
#include "../base/VectorView.hpp"
#include "../instance/Instance.hpp"

namespace cobra {

    // Simple generator of incremental numbers.
    class TimestampGenerator : private NonCopyable<TimestampGenerator> {
    public:
        TimestampGenerator() = default;
        inline unsigned long get() {
            return value;
        }
        inline void increment() {
            ++value;
        }

    private:
        unsigned long value = 0;
    };

    // Class representing a move generator or static move descriptor.
    class MoveGenerator : private NonCopyable<MoveGenerator> {
    public:
        MoveGenerator(int i, int j) : first_vertex(i), second_vertex(j) { }

        inline auto get_first_vertex() const {
            return first_vertex;
        }
        inline auto get_second_vertex() const {
            return second_vertex;
        }
        inline auto get_delta() const {
            return delta;
        }
        inline auto set_delta(double value) {
            delta = value;
        }
        inline auto get_heap_index() const {
            return heap_index;
        }
        inline auto set_heap_index(int index) {
            heap_index = index;
        }
        inline bool is_computed_for_ejch() const {
            return computed_for_ejch;
        }
        inline void set_computed_for_ejch(bool value) {
            computed_for_ejch = value;
        }

    private:
        int first_vertex;
        int second_vertex;
        double delta = 0.0;
        int heap_index = -1;
        bool computed_for_ejch = false;
    };

    struct MGCompare {
        auto operator()(MoveGenerator* mg1, MoveGenerator* mg2) {
            assert(mg1 && mg2);
            return mg1->get_delta() - mg2->get_delta();
        }
    };

    struct MGGetIdx {
        auto operator()(MoveGenerator* mg1) {
            assert(mg1);
            return mg1->get_heap_index();
        }
    };

    struct MGSetIdx {
        void operator()(MoveGenerator* mg1, int idx) {
            assert(mg1);
            mg1->set_heap_index(idx);
        }
    };

    struct MGUpdate {
        auto operator()(MoveGenerator* mg1, double delta) {
            assert(mg1);
            const auto res = mg1->get_delta() - delta;
            mg1->set_delta(delta);
            return res;
        }
    };

    // Heap data structure specialized to contain move generators.
    class MoveGeneratorsHeap
        : private NonCopyable<MoveGeneratorsHeap>
        , private BinaryHeap<MoveGenerator*, MGCompare, MGGetIdx, MGSetIdx, MGUpdate, -1> {

        typedef BinaryHeap<MoveGenerator*, MGCompare, MGGetIdx, MGSetIdx, MGUpdate> BHeap;

    public:
        MoveGeneratorsHeap() = default;
        MoveGeneratorsHeap(const MoveGeneratorsHeap& other) = delete;


        void reset() {
            BHeap::reset();
        }
        bool is_empty() const {
            return BHeap::empty();
        }
        void insert(MoveGenerator* mg) {
            BHeap::insert(mg);
        }
        MoveGenerator* get() {
            return BHeap::get();
        }
        void remove(int heap_index) {
            BHeap::remove(heap_index);
        }
        void change_value(int heap_index, double value) {
            BHeap::update(heap_index, value);
        };
        int size() const {
            return BHeap::size();
        };
        MoveGenerator* spy(int heap_index) {
            return BHeap::spy(heap_index);
        }

        static const int unheaped = -1;

    private:
        void dump() override {
            for (auto n = 0; n < size(); n++) {
                const auto& move = spy(n);
                std::cout << "[" << n << "] (" << move->get_first_vertex() << ", " << move->get_second_vertex()
                          << ") delta = " << move->get_delta() << " heap index = " << move->get_heap_index() << "\n";
            }
        }
    };

    // Forward declaration.
    class MoveGenerators;

    // Generic view over a set of move generators.
    class AbstractMoveGeneratorsView : private NonCopyable<AbstractMoveGeneratorsView> {

    public:
        AbstractMoveGeneratorsView(const Instance& instance_, std::function<std::vector<int>(int)> generator_)
            : generator(std::move(generator_)), instance(instance_) {
            active_move_generator_indices_involving.resize(instance.get_vertices_num());
            all_move_generator_indices_involving.resize(instance.get_vertices_num());
        }

        // Returns a function that returns for every vertex `i`, the set of vertices `j` defining move generators `(i, j)`.
        const std::function<std::vector<int>(int)>& get_generator() {
            return generator;
        }

        // Returns the indices of move generators `(i, j)` where `i = vertex` or `j = vertex`.
        const std::vector<int>& get_move_generator_indices_involving(int vertex) {
            return active_move_generator_indices_involving[vertex];
        }

        // During the construction of the `MoveGenerators` object, each move generator generated by the `generator` function is assigned a
        // unique index, which is its position in the `moves` vector in `MoveGenerators`. This function is called once this assignement is
        // concluded to nofify the view about the indices associated with move generators involving `vertex`.
        virtual void on_unique_index_assigned(int vertex, std::vector<int>& indices, MoveGenerators& moves) = 0;

        // Notifies that `MoveGenerators` has completely build the set of move generators associated with this view. Views can use this
        // callback to perform post-processing actions.
        virtual void on_build_completed(MoveGenerators& moves) = 0;

        // Changes the active move generators associated with `i = vertices[n], n=0, ..., len(vertices)` to be `percentage[i]`. Vertices in
        // `vertices_in_updated_moves` have been affected by this change.
        virtual void set_active_percentage(std::vector<double>& percentage, std::vector<int>& vertices, MoveGenerators& moves,
                                           SparseIntSet& vertices_in_updated_moves) = 0;

        // Returns the number of move generators associated with this view.
        virtual int size() = 0;

        // Returns whether the move generator indexed `move_idx` in `MoveGenerators` is active according to current sparsification level,
        // i.e., what was previously set with `set_active_percentage`.
        inline bool is_active(int move_idx) {
            assert(move_map.count(move_idx));
            const auto mapped_idx = move_map[move_idx];
            return active_in[mapped_idx].first || active_in[mapped_idx].second;
        }

    private:
        std::function<std::vector<int>(int)> generator;

    protected:
        const Instance& instance;

        // Every view keeps track of the indices of move generators in `MoveGenerators`, this allows to potentially have a single move
        // generator which is shared by multiple views.
        std::vector<std::vector<int>> all_move_generator_indices_involving;
        std::vector<std::vector<int>> active_move_generator_indices_involving;

        // Maps indices from `MoveGenerators` to contiguous indices for this specific view.
        std::unordered_map<int, int> move_map;

        // Keeps track for every local move index whether it is active in `i` or `j`.
        std::vector<std::pair<bool, bool>> active_in;

        // Maps indices from `MoveGenerators` into contiguous indices for this view, and initialize `active_in` by setting that every move
        // generator is not active in both `i` and `j`.
        void build_local_indices_and_setup_active_trackers() {
            for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {
                for (const auto idx : all_move_generator_indices_involving[i]) {
                    if (move_map.count(idx)) {
                        continue;
                    }
                    move_map.insert({idx, active_in.size()});
                    active_in.emplace_back(false, false);
                }
            }
        }

        // Sets that move generator `(i, j)` indexed `move_idx` in `MoveGenerators` is active in `vertex` (which could be `i` or `j`) ,
        // i.e., `active_move_generators_indices_involving[vertex]` contains `mapped_idx`.
        inline auto set_active_in(const MoveGenerator& move, int move_idx, int vertex) {
            assert(move_map.count(move_idx));
            const auto mapped_idx = move_map[move_idx];
            assert(vertex == move.get_first_vertex() || vertex == move.get_second_vertex());
            if (vertex == move.get_first_vertex()) {
                active_in[mapped_idx].first = true;
            } else {
                active_in[mapped_idx].second = true;
            }
        }

        // Sets that move generator `(i, j)` indexed `move_idx` in `MoveGenerators` is not active in `vertex` (which could be `i` or `j`) ,
        // i.e., `active_move_generators_indices_involving[vertex]` does not contains `mapped_idx`.
        inline auto set_not_active_in(const MoveGenerator& move, int move_idx, int vertex) {
            assert(move_map.count(move_idx));
            const auto mapped_idx = move_map[move_idx];
            assert(vertex == move.get_first_vertex() || vertex == move.get_second_vertex());
            if (vertex == move.get_first_vertex()) {
                active_in[mapped_idx].first = false;
            } else {
                active_in[mapped_idx].second = false;
            }
        }

        // Returns whether move generator `(i, j)` indexed `move_idx` in `MoveGenerators` is active in `vertex = i` or `vertex = j`.
        inline auto is_active_in(const MoveGenerator& move, int move_idx, int vertex) {
            assert(move_map.count(move_idx));
            const auto mapped_idx = move_map[move_idx];
            assert(vertex == move.get_first_vertex() || vertex == move.get_second_vertex());
            if (vertex == move.get_first_vertex()) {
                return active_in[mapped_idx].first;
            } else {
                return active_in[mapped_idx].second;
            }
        }

        // Returns whether move generator `(i, j)` indexed `move_idx` in `MoveGenerators` is active in `i` when `vertex = j` or viceversa.
        inline auto is_active_in_other(const MoveGenerator& move, int move_idx, int vertex) {
            assert(move_map.count(move_idx));
            const auto mapped_idx = move_map[move_idx];
            assert(vertex == move.get_first_vertex() || vertex == move.get_second_vertex());
            if (vertex == move.get_first_vertex()) {
                return active_in[mapped_idx].second;
            } else {
                return active_in[mapped_idx].first;
            }
        }
    };

    // Move generator containers. It manages the actual move generator objects required by the views.
    class MoveGenerators : private NonCopyable<MoveGenerators> {

    public:
        MoveGenerators(const Instance& instance_, std::vector<AbstractMoveGeneratorsView*>& views_)
            : instance(instance_), views(views_), heap(MoveGeneratorsHeap()), vertex_timestamp(instance_.get_vertices_num(), 0) {

            update_bits.resize(instance.get_vertices_num(), 2);

            // Simple hash function for a move generator `(i, j)`.
            struct pair_hash {
                auto operator()(const std::pair<int, int>& p) const -> size_t {
                    const auto prime = 31;
                    auto result = 1;
                    result = prime * result + p.first;
                    result = prime * result + p.second;
                    return std::hash<int>()(result);
                }
            };

            // Different views may define the same move generator. For efficiency purposes, a single move generator is created and referred
            // by all views.
            auto unique_moves = std::unordered_map<std::pair<int, int>, int, pair_hash>();

            for (auto& view : views) {

                const auto& generator = view->get_generator();

                for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {

                    // Generate vertices `j` that toghether with `i` define move generators `(i, j)` and `(j, i)`.
                    const auto& endpoints = generator(i);

                    auto move_indices_in_moves = std::vector<int>();
                    move_indices_in_moves.reserve(endpoints.size());

                    for (auto j : endpoints) {

                        auto a = i;
                        auto b = j;
                        if (b < a) {
                            std::swap(a, b);
                        }

                        if (unique_moves.count({a, b})) {

                            const auto move_index = unique_moves[{a, b}];

                            move_indices_in_moves.emplace_back(move_index);

                        } else {

                            const auto move_index = moves.size();
                            move_indices_in_moves.emplace_back(move_index);

                            moves.emplace_back(a, b);
                            moves.emplace_back(b, a);
                            edge_costs.emplace_back(instance.get_cost(a, b));

                            unique_moves[{a, b}] = move_index;
                        }
                    }

                    view->on_unique_index_assigned(i, move_indices_in_moves, *this);
                }

                view->on_build_completed(*this);
            }

            this->move_generator_indices_involving.resize(instance.get_vertices_num());
            this->curr_percentage.resize(instance.get_vertices_num(), 0.0);
        }

        virtual ~MoveGenerators() { }

        // Returns move generator indexed `idx`.
        inline MoveGenerator& get(int idx) {
            assert(idx >= 0 && idx < static_cast<int>(moves.size()));
            return moves[idx];
        }

        // Set the active move generators considered during local search by updating the active move generators for each view. Vector
        // `percentage` must have the same size of the instance and must contain numbers between 0 and 1. Vector `vertices` contains only
        // vertices for which some change is required. Vertices that are not affected by the procedure are removed by `vertices`. Callers
        // must not rely on the order of `vertices`.
        void set_active_percentage(std::vector<double>& percentage, std::vector<int>& vertices) {

            assert(static_cast<int>(percentage.size()) == instance.get_vertices_num());

            // Identify vertices that require an update.
            for (auto n = 0u; n < vertices.size();) {
                const auto vertex = vertices[n];
                if (std::fabs(percentage[vertex] - curr_percentage[vertex]) < 0.01) {
                    // Remove a vertex if it does not need to be updated.
                    std::swap(vertices[n], vertices[vertices.size() - 1]);
                    vertices.pop_back();
                } else {
                    n++;
                }
            }

            if (vertices.empty()) {
                return;
            }

            // For each view, set or unset move generators involving each 'vertex' according to percentage[vertex].
            // Collect all vertices involving moves that are set or unset.
            auto vertices_in_updated_moves = SparseIntSet(instance.get_vertices_num());
            for (auto& view : views) {
                view->set_active_percentage(percentage, vertices, *this, vertices_in_updated_moves);
            }

            // The previous view->set_active_percentage(...) for vertex `i` may cause of of the following.
            //
            // DIRECT UPDATE
            // We are adding or removing move generator `(i, j)` to the list of move generators involving `i`
            //
            // INDIRECT UPDATE
            // 1. We are adding move generator `(i, j)` to the list of move generators involving `j` because of the direct update of vertex
            // `i` (addition)
            // 2. We are setting move generator `(i, j)` as not active since it was only active for `j` due to being active for `i`
            // (removal). For case 2 we do not have an efficient way to remove it from its view list but we are only doing some cleanup on
            // the unified list of move generators here. In particular, we are only adding to this list move generators that are active in
            // both `i` and `j`.
            // Note that this latter approach causes views to always stay in possibly inconsistent states and thus they should never be
            // accessed directly.
            auto unique_move_generators = std::vector<int>();
            auto unique_endpoints = SparseIntSet(instance.get_vertices_num());
            for (const auto vertex : vertices_in_updated_moves.get_elements()) {
                unique_move_generators.clear();
                unique_endpoints.clear();
                for (auto& view : views) {
                    for (auto move_idx : view->get_move_generator_indices_involving(vertex)) {
                        if (view->is_active(move_idx)) {

                            if (vertex != moves[move_idx].get_first_vertex()) {
                                move_idx = get_twin_move_generator_index(move_idx);
                            }

                            const auto& move = moves[move_idx];
                            int other_vertex = move.get_second_vertex();

                            if (!unique_endpoints.contains(other_vertex)) {
                                unique_endpoints.insert_without_checking_existance(other_vertex);
                                unique_move_generators.push_back(move_idx);
                            }
                        }
                    }
                }

                move_generator_indices_involving[vertex] = unique_move_generators;
            }

            // Finally, update the stored percentages.
            for (const auto vertex : vertices) {
                curr_percentage[vertex] = percentage[vertex];
            }
        }

        // Returns the heap used for storing move generators during local search applications.
        inline MoveGeneratorsHeap& get_heap() {
            return heap;
        }

        // Returns the index of `(i, j)` given the index of `(j, i)` and viceversa.
        static inline int get_twin_move_generator_index(int index) {
            return index ^ 1;
        }

        // Returns a index of `(i, j)` given the index of `(i, j)` or the index of `(j, i)`.
        static inline auto get_base_move_generator_index(int index) {
            return index & ~1;
        }

        // Returns move generators `(i, j)` where `i == vertex`.
        inline const auto& get_move_generator_indices_involving_1st(int vertex) const {
            return move_generator_indices_involving[vertex];
        }

        // Returns move generators `(i, j)` where `j == vertex`.
        inline auto get_move_generator_indices_involving_2nd(int vertex) const {
            const auto& v = move_generator_indices_involving[vertex];
            return VectorView<decltype(v.begin()), twin_functor>(v.begin(), v.end());
        }

        // Returns move generators `(i, j)`.
        inline auto get_move_generator_indices_involving(int vertex) const {
            const auto& v = move_generator_indices_involving[vertex];
            return VectorView<decltype(v.begin()), base_functor>(v.begin(), v.end());
        }

        // Returns a timestamp generator used by local search operators to mark move generators associated with a vertex as updated.
        inline TimestampGenerator& get_timestamp_generator() {
            return timegen;
        }

        // Returns a vector of timestamp associated with every vertex. Used by local search operators to mark move generators as updated.
        inline std::vector<unsigned long>& get_vertex_timestamp() {
            return vertex_timestamp;
        }

        // Returns the update bits data structure.
        inline Flat2DVector<bool>& get_update_bits() {
            return update_bits;
        }

        // Return the instance arc cost associated with the move generator. It is stored here since pervasively used, so that it can be
        // retrieved in constant time instead of possibly hitting a cache miss in the `Instance` class.
        inline double get_edge_cost(const MoveGenerator& move) {
            const int index = (&move - moves.data()) / 2;
            return edge_costs[index];
        }

        // Number of move generator objects stored in memory.
        inline size_t size() {
            return moves.size();
        }

    private:
        const Instance& instance;
        std::vector<MoveGenerator> moves;

        // Edge cost of the associated move generator. Since costs are symmetric and we are storing both move generators (ij) and (ji)
        // consecutively, the vector edge_cost contains c(ij)=c(ji) once only.
        std::vector<double> edge_costs;

        std::vector<AbstractMoveGeneratorsView*>& views;
        MoveGeneratorsHeap heap;

        // Move generatro indices summarized from every view without duplicates.
        std::vector<std::vector<int>> move_generator_indices_involving;

        // Keeps track up the current percentage associated with each vertex.
        std::vector<double> curr_percentage;

        // The `updated_bits` are used by local search operators to identify whether `(i, j)`, `(j, i)` or both require an update upon a
        // move execution.
        Flat2DVector<bool> update_bits;

        // Used by local search operators to identify whether move generators of a given vertex have already been updated.
        std::vector<unsigned long> vertex_timestamp;
        TimestampGenerator timegen;
    };

    // Move generators view based on the k nearest neighbors.
    class KNeighborsMoveGeneratorsView : public AbstractMoveGeneratorsView {

    public:
        KNeighborsMoveGeneratorsView(const Instance& inst, int k)
            : AbstractMoveGeneratorsView(inst,
                                         [&inst, k](auto i) -> std::vector<int> {
                                             auto endpoints = std::vector<int>();

                                             const auto max_neighbors = std::min(k, inst.get_vertices_num() - 1);

                                             for (auto n = 1, added = 0; added < max_neighbors; n++) {

                                                 assert(n < static_cast<int>(inst.get_neighbors_of(i).size()));

                                                 auto j = inst.get_neighbors_of(i)[n];

                                                 endpoints.emplace_back(j);
                                                 added++;
                                             }

                                             return endpoints;
                                         })
            , max_neighbors_num(std::min(k, inst.get_vertices_num() - 1))  // Skip self-moves.
        {
            curr_neighbors_num.resize(inst.get_vertices_num(), 0);
        }


        int size() override {
            auto set = std::set<int>();
            for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {
                for (auto id : all_move_generator_indices_involving[i]) {
                    set.insert(id);
                }
            }
            // Multiply by 2 since we are storing a single copy between `(i, j)` and `(j, i)`.
            return 2 * static_cast<int>(set.size());
        }

    private:
        void on_unique_index_assigned([[maybe_unused]] int vertex, std::vector<int>& indices, MoveGenerators& moves) override {
            for (const auto index : indices) {
                const auto& move = moves.get(index);
                const auto i = move.get_first_vertex();
                const auto j = move.get_second_vertex();
                assert(i == vertex || j == vertex);
                all_move_generator_indices_involving[i].emplace_back(index);
                all_move_generator_indices_involving[j].emplace_back(index);
            }
        }
        void on_build_completed(MoveGenerators& moves) override {

            for (auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {

                auto& indices = all_move_generator_indices_involving[i];

                auto set = std::unordered_set<int>();

                set.insert(indices.begin(), indices.end());
                indices.clear();
                indices.insert(indices.begin(), set.begin(), set.end());
                indices.shrink_to_fit();
                std::sort(indices.begin(), indices.end(), [&moves, this](auto a, auto b) {
                    const auto& a_move = moves.get(a);
                    const auto a_cost = this->instance.get_cost(a_move.get_first_vertex(), a_move.get_second_vertex());

                    const auto& b_move = moves.get(b);
                    const auto b_cost = this->instance.get_cost(b_move.get_first_vertex(), b_move.get_second_vertex());

                    return a_cost < b_cost;
                });
            }

            build_local_indices_and_setup_active_trackers();
        }
        void set_active_percentage(std::vector<double>& percentage, std::vector<int>& vertices, MoveGenerators& moves,
                                   SparseIntSet& vertices_in_updated_moves) override {

            auto vertices_to_update = std::vector<int>();

            // For each vertex, identify the move generators that are affected, i.e., need to be set as active or not active.
            for (const auto vertex : vertices) {

                const auto new_num = static_cast<int>(std::round(percentage[vertex] * static_cast<double>(max_neighbors_num)));
                assert(new_num <= static_cast<int>(all_move_generator_indices_involving[vertex].size()));

                if (new_num == curr_neighbors_num[vertex]) {
                    continue;
                }

                vertices_to_update.push_back(vertex);

                // Set as active or not active the move generators associated to `vertex` according to `new_num`.
                // While doing this, keep track of the vertices for which at least one move generator has been updated. This info will be
                // later used by the `MoveGenerators` object to perform a selective update of the necessary data structures.
                if (new_num < curr_neighbors_num[vertex]) {
                    // Removal.
                    for (auto n = new_num; n < curr_neighbors_num[vertex]; n++) {
                        const auto move_idx = all_move_generator_indices_involving[vertex][n];
                        const auto& move = moves.get(move_idx);
                        this->set_not_active_in(move, move_idx, vertex);
                        vertices_in_updated_moves.insert(move.get_first_vertex());
                        vertices_in_updated_moves.insert(move.get_second_vertex());
                    }
                } else {
                    // Addition.
                    for (auto n = curr_neighbors_num[vertex]; n < new_num; n++) {
                        const auto move_idx = all_move_generator_indices_involving[vertex][n];
                        const auto& move = moves.get(move_idx);
                        this->set_active_in(move, move_idx, vertex);
                        vertices_in_updated_moves.insert(move.get_first_vertex());
                        vertices_in_updated_moves.insert(move.get_second_vertex());
                    }
                }
                curr_neighbors_num[vertex] = new_num;
            }

            // Note that before actually inserting the move generator indices in the list of move generators involving a vertex
            // we have to update the active/not active data structures that will be used in the following

            // Define the current list of active move generators per vertex.
            for (const auto vertex : vertices_to_update) {

                active_move_generator_indices_involving[vertex].clear();

                auto n = 0;

                // Scan move generators for the current number of active neighbors.
                for (; n < curr_neighbors_num[vertex]; n++) {
                    const auto idx = all_move_generator_indices_involving[vertex][n];
                    const auto& move = moves.get(idx);

                    const auto i = move.get_first_vertex();
                    const auto j = move.get_second_vertex();

                    assert(i == vertex || j == vertex);

                    assert(moves.get(idx + 1).get_first_vertex() == j);
                    assert(moves.get(idx + 1).get_second_vertex() == i);

                    assert(moves.get(idx).get_first_vertex() == i);
                    assert(moves.get(idx).get_second_vertex() == j);

                    assert(i < j);

                    if (vertex == i) {
                        // Processing `(i = vertex, j)` for list `active_move_generator_indices_involving[i]`.

                        active_move_generator_indices_involving[i].emplace_back(idx);

                        // Add also for vertex `j`, if not already there.
                        if (!is_active_in_other(move, idx, vertex)) {
                            active_move_generator_indices_involving[j].emplace_back(idx);
                        }

                    } else {
                        // Processing `(i, j = vertex)` for list `active_move_generator_indices_involving[j]`.

                        active_move_generator_indices_involving[j].emplace_back(idx);

                        // Add also for vertex `i`, if not already there.
                        if (!is_active_in_other(move, idx, vertex)) {
                            active_move_generator_indices_involving[i].emplace_back(idx);
                        }
                    }
                }

                // Scan remaining move generators since we might need to add some that are active in the other vertex and were not affected.
                for (; n < static_cast<int>(all_move_generator_indices_involving[vertex].size()); n++) {
                    const auto idx = all_move_generator_indices_involving[vertex][n];
                    const auto& move = moves.get(idx);

#ifndef NDEBUG
                    const auto i = move.get_first_vertex();
                    const auto j = move.get_second_vertex();
#endif

                    assert(i == vertex || j == vertex);

                    assert(moves.get(idx + 1).get_first_vertex() == j);
                    assert(moves.get(idx + 1).get_second_vertex() == i);

                    assert(moves.get(idx).get_first_vertex() == i);
                    assert(moves.get(idx).get_second_vertex() == j);

                    assert(i < j);

                    // Add move generators that are active in the other vertex.
                    if (is_active_in_other(move, idx, vertex)) {
                        active_move_generator_indices_involving[vertex].emplace_back(idx);
                    }
                }
            }
        }

        // Max number of neighbors that can be used.
        const int max_neighbors_num;

        // Current number of neighbors used.
        std::vector<int> curr_neighbors_num;
    };


}  // namespace cobra

#endif