#pragma once

#include <array>
#include <stdexcept>
#include <type_traits>

constexpr size_t variant_npos = -1;

template <class... Args> class variant;

template <size_t, typename> struct variant_alternative;

template <typename visitor_type, typename... variant_types>
constexpr decltype(auto) visit(visitor_type &&visitor, variant_types &&...variants);

struct in_place_t {
    explicit in_place_t() = default;
};

template <class> struct in_place_type_t { explicit in_place_type_t() = default; };

template <std::size_t> struct in_place_index_t { explicit in_place_index_t() = default; };

template <class t> struct variant_size;

class bad_variant_access : public std::exception {
public:
    bad_variant_access() = default;

    [[nodiscard]] const char *what() const noexcept override { return reason; }

    explicit bad_variant_access(const char *reason) noexcept : reason(reason) {}

private:
    const char *reason = "bad variant access";
};

constexpr in_place_t in_place{};

template <class t> constexpr in_place_type_t<t> in_place_type{};

template <std::size_t I> constexpr in_place_index_t<I> in_place_index{};

template <size_t Np, typename First, typename... Rest>
struct variant_alternative<Np, variant<First, Rest...>> : variant_alternative<Np - 1, variant<Rest...>> {};

template <typename First, typename... Rest> struct variant_alternative<0, variant<First, Rest...>> {
    using type = First;
};

template <size_t Np, typename Variant> using variant_alternative_t = typename variant_alternative<Np, Variant>::type;

template <std::size_t idx, class t> struct variant_alternative<idx, const t> {
    using type = std::add_const_t<typename variant_alternative<idx, t>::type>;
};

template <std::size_t idx, class t> struct variant_alternative<idx, volatile t> {
    using type = std::add_volatile_t<typename variant_alternative<idx, t>::type>;
};

template <std::size_t idx, class t> struct variant_alternative<idx, const volatile t> {
    using type = std::add_cv_t<typename variant_alternative<idx, t>::type>;
};

template <size_t idx, class type> using variant_alternative_t = typename variant_alternative<idx, type>::type;

template <class... types>
struct variant_size<variant<types...>> : std::integral_constant<std::size_t, sizeof...(types)> {};

template <class t> struct variant_size : variant_size<std::remove_cv_t<t>> {};

template <class t> constexpr std::size_t variant_size_v = variant_size<t>::value;

namespace internal {

    template <class... types> struct type_pack { static constexpr size_t size = sizeof...(types); };

    template <typename visitor_type, typename... variant_types>
    constexpr decltype(auto) raw_visit(visitor_type &&visitor, variant_types &&...variants);

    template <class t, class... types> struct index_of : std::integral_constant<std::size_t, 0> {};

    template <class, class> struct index_of_pack;

    template <class t, class... pack_types>
    struct index_of_pack<t, type_pack<pack_types...>> : index_of<t, pack_types...> {};

    template <typename type, typename... types> constexpr size_t index_of_v = index_of<type, types...>::value;

    template <typename type, typename pack> constexpr size_t index_of_pack_v = index_of_pack<type, pack>::value;

    template <class t, class F, class... Rest>
    struct index_of<t, F, Rest...>
            : std::integral_constant<std::size_t, std::is_same_v<t, F> ? 0 : index_of_v<t, Rest...> + 1> {};

    template <class first_type, class snd_type> struct union_storage {
        union {
            first_type first;
            snd_type second;
        };

        constexpr union_storage() noexcept(std::is_nothrow_default_constructible_v<first_type>) : second(){};
        template <class... types>
        constexpr explicit union_storage(in_place_index_t<0>,
                                         types &&...t) noexcept(std::is_nothrow_constructible_v<first_type, types...>)
                : first(in_place_index<0>, std::forward<types>(t)...) {}
        template <std::size_t idx, class... types>
        constexpr explicit union_storage(in_place_index_t<idx>,
                                         types &&...t) noexcept(std::is_nothrow_constructible_v<snd_type, types...>)
                : second(in_place_index_t<idx - 1>{}, std::forward<types>(t)...) {}

        constexpr union_storage(union_storage &&) = default;
        constexpr union_storage(const union_storage &) = default;
        constexpr union_storage &operator=(union_storage &&) = default;
        constexpr union_storage &operator=(const union_storage &) = default;
    };

    template <class type, bool = std::is_trivially_destructible_v<type>> struct element_storage;

    template <class type> struct element_storage<type, true> {
        constexpr element_storage() = default;

        template <class ctor_type> constexpr explicit element_storage(ctor_type &&u) : value_(std::forward<ctor_type>(u)) {}

        template <class... emp_type>
        constexpr explicit element_storage(in_place_index_t<0>,
                                           emp_type &&...h) noexcept(std::is_nothrow_constructible_v<type, emp_type...>)
                : value_(std::forward<emp_type>(h)...) {}

        constexpr element_storage(const element_storage &) = default;

        constexpr element_storage(element_storage &&) = default;

        constexpr element_storage &operator=(element_storage &&) = default;
        constexpr element_storage &operator=(const element_storage &) = default;

        constexpr type &value() & { return value_; }

        constexpr const type &value() const & { return value_; }

        constexpr type &&value() && { return std::move(value_); }

        constexpr const type &&value() const && { return std::move(value_); }

    private:
        type value_;
    };

    template <class type> struct aligned_element_storage {
        template <class... U>
        constexpr explicit aligned_element_storage(U &&...t) noexcept(std::is_nothrow_constructible_v<type, U...>) {
            static_assert(sizeof(value) == sizeof(type));
            ::new (&value) type(std::forward<U>(t)...);
        }

        type *get() { return static_cast<type *>(static_cast<void *>(&value)); }

        const type *get() const { return static_cast<const type *>(static_cast<const void *>(&value)); }

        alignas(alignof(type)) char value[sizeof(type)]{};
    };

    template <class t> struct element_storage<t, false> {
        constexpr element_storage() = default;

        template <class U> constexpr explicit element_storage(U &&u) : value_(std::forward<U>(u)) {}

        template <class... H>
        constexpr explicit element_storage(in_place_index_t<0>, H &&...h) : value_(std::forward<H>(h)...) {}

        t &value() & { return *value_.get(); }

        const t &value() const & { return *value_.get(); }

        t &&value() && { return std::move(*value_.get()); }

        const t &&value() const && { return std::move(*value_.get()); }

    private:
        aligned_element_storage<t> value_;
    };

    template <class> struct is_element_storage_impl { static constexpr bool value = false; };

    template <class T> struct is_element_storage_impl<element_storage<T>> { static constexpr bool value = true; };

    template <class T> struct is_element_storage : is_element_storage_impl<std::decay_t<T>> {};

    template <class type, typename = std::enable_if_t<is_element_storage<type>::value>>
    constexpr decltype(auto) get(in_place_index_t<0>, type &&v) {
        return std::forward<type>(v).value();
    }

    template <std::size_t I, class union_storage, typename = std::enable_if_t<!is_element_storage<union_storage>::value>>
    constexpr decltype(auto) get(in_place_index_t<I>, union_storage &&h) {
        if constexpr (I == 0)
            return get(in_place_index<I>, std::forward<union_storage>(h).first);
        else
            return get(in_place_index<I - 1>, std::forward<union_storage>(h).second);
    }

    template <typename t> constexpr t &&at_impl(t &&elem) { return std::forward<t>(elem); }

    template <typename type, typename... indices> constexpr auto &&at_impl(type &&elems, std::size_t i, indices... is) {
        return at_impl(std::forward<type>(elems)[i], is...);
    }

    template <typename type, typename... indices> constexpr auto &&at(type &&elems, indices... is) {
        return at_impl(std::forward<type>(elems), is...);
    }

    template <typename func_type, typename... variant_types, std::size_t... indices>
    constexpr decltype(auto) make_raw_vtable(std::index_sequence<indices...>) {
        struct dispatcher {
            static constexpr decltype(auto) dispatch(func_type f, variant_types... vs) {
                return static_cast<func_type>(f)(std::integral_constant<std::size_t, indices>{}...);
            }
        };
        return &dispatcher::dispatch;
    }

    template <typename... types> using make_array_elem = std::common_type<types...>;

    template <typename... types>
    constexpr std::array<typename make_array_elem<types...>::type, sizeof...(types)> make_array(types &&...t) {
        return {{std::forward<types>(t)...}};
    }

    template <typename func_type, typename... variant_types, std::size_t... indices_i, std::size_t... indices_s,
            typename... ind_types>
    constexpr auto make_raw_vtable(std::index_sequence<indices_i...>, std::index_sequence<indices_s...>, ind_types... ls) {
        return make_array(
                make_raw_vtable<func_type, variant_types...>(std::index_sequence<indices_i..., indices_s>{}, ls...)...);
    }

    template <typename func_type, typename... variant_types> constexpr auto create_raw_vtable() {
        return make_raw_vtable<func_type, variant_types...>(
                std::index_sequence<>{}, std::make_index_sequence<variant_size_v<std::decay_t<variant_types>>>{}...);
    }

    template <class type_arg, class... types> struct type_pack<type_arg, types...> {
        static constexpr size_t size = sizeof...(types) + 1;
        using type = union_storage<element_storage<type_arg>, typename type_pack<types...>::type>;
    };

    template <class type_arg> struct type_pack<type_arg> {
        static constexpr size_t size = 1;
        using type = union_storage<element_storage<type_arg>, char>;
    };

    template <class type> struct array_helper { type x[1]; };

    template <class return_type, class t, class tj, bool = std::is_same_v<std::remove_cv_t<t>, bool>, typename = void>
    struct meta_function {
        void apply(); // SFINAE
    };

    template <class return_type, class ti, class tj>
    struct meta_function<return_type, ti, tj, false, std::void_t<decltype(array_helper<ti>{{std::declval<tj>()}})>> {
        static return_type apply(ti);
    };

    template <class return_type, class ti, class tj>
    struct meta_function<return_type, ti, tj, true,
            std::enable_if_t<std::is_same_v<std::remove_cv_t<std::remove_reference_t<tj>>, bool>>> {
        static return_type apply(ti);
    };

    template <class... ts> struct overloaded : ts... { using ts::apply...; };

    template <class... Args> struct choose_impl;

    template <std::size_t... indices, class t, class... types>
    struct choose_impl<std::index_sequence<indices...>, t, types...>
            : overloaded<meta_function<std::integral_constant<std::size_t, indices>, types, t>...> {
        using overloaded<meta_function<std::integral_constant<std::size_t, indices>, types, t>...>::apply;
    };

    template <class... Args> struct chase_type;

    template <class t, class... Args>
    struct chase_type<t, Args...> : choose_impl<std::make_index_sequence<sizeof...(Args)>, t, Args...> {
        using choose_impl<std::make_index_sequence<sizeof...(Args)>, t, Args...>::apply;
    };

    template <class t, class... types> using choose_type = decltype(chase_type<t, types...>::apply(std::declval<t>()));

    template <class t, class B, typename = void> struct chase_index : std::integral_constant<std::size_t, variant_npos> {};

    template <class t, class... types>
    struct chase_index<t, type_pack<types...>, std::void_t<choose_type<t, types...>>> : choose_type<t, types...> {};

    template <class t, class... types> constexpr auto choose_index = chase_index<t, type_pack<types...>>::value;

    template <class t> struct is_inplace_type : std::false_type {};

    template <class t> struct is_inplace_type<in_place_type_t<t>> : std::true_type {};

    template <std::size_t I> struct is_inplace_type<in_place_index_t<I>> : std::true_type {};

    template <typename t, typename tuple> struct count_same_types;

    template <typename t, typename tuple> constexpr size_t tuple_count_v = count_same_types<t, tuple>::value;

    template <typename t, typename... types>
    struct count_same_types<t, type_pack<types...>> : std::integral_constant<size_t, 0> {};

    template <typename t, typename first_type, typename... rest_types>
    struct count_same_types<t, type_pack<first_type, rest_types...>>
            : std::integral_constant<size_t, tuple_count_v<t, type_pack<rest_types...>> + std::is_same_v<t, first_type>> {};

    template <typename type, typename... types>
    constexpr bool is_occur_once = tuple_count_v<type, type_pack<types...>> == 1;

    template <typename type, typename pack> constexpr bool is_occur_once_in_pack = tuple_count_v<type, pack> == 1;

    template <class> struct extract_type_pack;

    template <typename... var_types> struct extract_type_pack<variant<var_types...>> {
        using type = type_pack<var_types...>;
    };

    template <typename type>
    struct extract_type_pack : extract_type_pack<std::remove_cv_t<std::remove_reference_t<type>>> {};

    template <typename type> using extract_type_pack_t = typename extract_type_pack<type>::type;

    template <class... types, class type> decltype(auto) variant_cast(type &&t) {
        if constexpr (std::is_lvalue_reference_v<type>) {
            if constexpr (std::is_const_v<std::remove_reference_t<type>>)
                return *static_cast<const variant<types...> *>(static_cast<const void *>(&t));
            else
                return *static_cast<variant<types...> *>(static_cast<void *>(&t));
        } else
            return std::move(*static_cast<variant<types...> *>(static_cast<void *>(&t)));
    }

    template <class t, class... types> struct traits {
        static constexpr bool ENABLE_DEFAULT_CTOR = (std::is_default_constructible_v<t>);
        static constexpr bool ENABLE_COPY_CTOR =
                (std::is_copy_constructible_v<t> && ... && std::is_copy_constructible_v<types>);
        static constexpr bool ENABLE_MOVE_CTOR =
                (std::is_move_constructible_v<t> && ... && std::is_move_constructible_v<types>);

        static constexpr bool ENABLE_COPY_ASSIGN =
                ENABLE_COPY_CTOR && (std::is_copy_assignable_v<t> && ... && std::is_copy_assignable_v<types>);
        static constexpr bool ENABLE_MOVE_ASSIGN =
                ENABLE_MOVE_CTOR && (std::is_move_assignable_v<t> && ... && std::is_move_assignable_v<types>);

        static constexpr bool IS_TRIVIAL_DTOR =
                (std::is_trivially_destructible_v<types> && ... && std::is_trivially_destructible_v<t>);
        static constexpr bool IS_TRIVIAL_MOVE_CTOR =
                (std::is_trivially_move_constructible_v<types> && ... && std::is_trivially_move_constructible_v<t>);
        static constexpr bool IS_TRIVIAL_COPY_CTOR =
                (std::is_trivially_copy_constructible_v<types> && ... && std::is_trivially_copy_constructible_v<t>);
        static constexpr bool IS_TRIVIAL_MOVE_ASSIGN =
                IS_TRIVIAL_DTOR && IS_TRIVIAL_MOVE_CTOR &&
                (std::is_trivially_move_assignable_v<types> && ... && std::is_trivially_move_assignable_v<t>);
        static constexpr bool IS_TRIVIAL_COPY_ASSIGN =
                IS_TRIVIAL_DTOR && IS_TRIVIAL_COPY_CTOR &&
                (std::is_trivially_copy_assignable_v<types> && ... && std::is_trivially_copy_assignable_v<t>);

        static constexpr bool IS_NOTHROW_DEFAULT_CTOR = std::is_nothrow_default_constructible_v<t>;
        static constexpr bool IS_NOTHROW_COPY_CTOR = false;
        static constexpr bool IS_NOTHROW_MOVE_CTOR =
                (std::is_nothrow_move_constructible_v<types> && ... && std::is_nothrow_move_constructible_v<t>);
        static constexpr bool IS_NOTHROW_MOVE_ASSIGN =
                IS_NOTHROW_MOVE_CTOR && (std::is_nothrow_move_assignable_v<types> && ... && std::is_nothrow_move_assignable_v<t>);
        static constexpr bool IS_NOTHROW_COPY_ASSIGN = false;
    };

    namespace base {
        template <bool, class... types> struct storage;

        template <bool, class... types> struct move_assign;

        template <bool, class... types> struct move_ctor;

        template <bool, class... types> struct copy_assign;

        template <bool, class... types> struct copy_ctor;

        template <class... types> struct storage<false, types...> {
            using value_type = typename internal::type_pack<types...>::type;
            value_type value_;
            std::size_t index_{};

            constexpr storage() : index_(variant_npos){};

            constexpr storage(const storage &other) = default;

            template <class t, class... Args>
            constexpr explicit storage(in_place_type_t<t>, Args &&...args)
                    : storage(in_place_index<internal::choose_index<t, types...>>, std::forward<Args>(args)...) {}

            template <class t, class U, class... Args>
            constexpr explicit storage(in_place_type_t<t>, std::initializer_list<U> il, Args &&...args)
                    : storage(in_place_index<internal::choose_index<t, types...>>, il, std::forward<Args>(args)...) {}

            template <std::size_t I, class... Args>
            constexpr explicit storage(in_place_index_t<I>, Args &&...args)
                    : value_(in_place_index<I>, std::forward<Args>(args)...), index_(I) {}

            template <std::size_t I, class U, class... Args>
            constexpr explicit storage(in_place_index_t<I>, std::initializer_list<U> il, Args &&...args)
                    : value_(in_place_index<I>, il, std::forward<Args>(args)...), index_(I) {}

            constexpr storage &operator=(storage &&) = default;

            constexpr storage &operator=(const storage &rhs) = default;

            [[nodiscard]] constexpr bool is_valid() const noexcept { return index_ != variant_npos; }

            void reset() {
                if (!is_valid())
                    return;

                raw_visit([this](auto index) { std::destroy_at(std::addressof(get<index>(variant_cast<types...>(*this)))); },
                          variant_cast<types...>(*this));

                index_ = variant_npos;
            }

            ~storage() { reset(); }
        };

        template <class... types> struct storage<true, types...> {
            using value_type = typename internal::type_pack<types...>::type;
            value_type value_;
            std::size_t index_{};

            constexpr storage() : index_(variant_npos) {}

            constexpr storage(const storage &other) = default;

            constexpr storage(storage &&other) = default;

            template <class t, class... Args>
            constexpr explicit storage(in_place_type_t<t>, Args &&...args)
                    : storage(in_place_index<internal::choose_index<t, types...>>, std::forward<Args>(args)...) {}

            template <class t, class U, class... Args>
            constexpr explicit storage(in_place_type_t<t>, std::initializer_list<U> il, Args &&...args)
                    : storage(in_place_index<internal::choose_index<t, types...>>, il, std::forward<Args>(args)...) {}

            template <std::size_t I, class... Args>
            constexpr explicit storage(in_place_index_t<I>, Args &&...args)
                    : value_(in_place_index<I>, std::forward<Args>(args)...), index_(I) {}

            template <std::size_t I, class U, class... Args>
            constexpr explicit storage(in_place_index_t<I>, std::initializer_list<U> il, Args &&...args)
                    : value_(in_place_index<I>, il, std::forward<Args>(args)...), index_(I) {}

            constexpr storage &operator=(storage &&rhs) = default;

            constexpr storage &operator=(const storage &rhs) = default;

            [[nodiscard]] constexpr bool is_valid() const noexcept { return index_ != variant_npos; }

            void reset() {
                if (!is_valid())
                    return;

                raw_visit([this](auto index) { std::destroy_at(std::addressof(get<index>(variant_cast<types...>(*this)))); },
                          variant_cast<types...>(*this));

                index_ = variant_npos;
            }
        };

        namespace alias {
            template <class... types> using storage = base::storage<traits<types...>::IS_TRIVIAL_DTOR, types...>;

            template <class... types> using move_assign = base::move_assign<traits<types...>::IS_TRIVIAL_MOVE_ASSIGN, types...>;

            template <class... types> using move_ctor = base::move_ctor<traits<types...>::IS_TRIVIAL_MOVE_CTOR, types...>;

            template <class... types> using copy_assign = base::copy_assign<traits<types...>::IS_TRIVIAL_COPY_ASSIGN, types...>;

            template <class... types> using copy_ctor = base::copy_ctor<traits<types...>::IS_TRIVIAL_COPY_CTOR, types...>;
        } // namespace alias

        template <class... types> struct move_assign<true, types...> : alias::storage<types...> {
            using base = alias::storage<types...>;
            using base::base;
        };

        template <class... types> struct move_assign<false, types...> : alias::storage<types...> {
            using base = alias::storage<types...>;
            using base::base;

            constexpr move_assign() = default;
            constexpr move_assign(move_assign const &) = default;
            constexpr move_assign(move_assign &&rhs) = default;
            constexpr move_assign &operator=(move_assign &&rhs) noexcept(traits<types...>::IS_NOTHROW_MOVE_ASSIGN) {
                raw_visit(
                        [this, &rhs](auto rhs_index) {
                            if constexpr (rhs_index != variant_npos) {
                                if (this->index_ == rhs_index) {
                                    get<rhs_index>(variant_cast<types...>(*this)) = std::move(get<rhs_index>(variant_cast<types...>(rhs)));
                                } else {
                                    variant_cast<types...>(*this).template emplace<rhs_index>(
                                            std::move(get<rhs_index>(variant_cast<types...>(rhs))));
                                }
                            } else {
                                this->reset();
                            }
                        },
                        variant_cast<types...>(rhs));

                return *this;
            }
            constexpr move_assign &operator=(move_assign const &) = default;
        };

        template <class... types> struct move_ctor<true, types...> : alias::move_assign<types...> {
            using base = alias::move_assign<types...>;
            using base::base;
        };

        template <class... types> struct move_ctor<false, types...> : alias::move_assign<types...> {
            using base = alias::move_assign<types...>;
            using base::base;

            constexpr move_ctor() = default;
            constexpr move_ctor(move_ctor const &) = default;
            constexpr move_ctor(move_ctor &&other) noexcept(traits<types...>::IS_NOTHROW_MOVE_CTOR) {
                using type = std::remove_reference_t<decltype(other.value_)>;

                this->index_ = other.index_;
                raw_visit(
                        [this, &other](auto index) {
                            ::new (std::addressof(this->value_))
                                    type(in_place_index<index>, std::move(get<index>(variant_cast<types...>(other))));
                        },
                        variant_cast<types...>(other));
            }
            constexpr move_ctor &operator=(move_ctor &&) = default;
            constexpr move_ctor &operator=(move_ctor const &) = default;
        };

        template <class... types> struct copy_assign<true, types...> : alias::move_ctor<types...> {
            using base = alias::move_ctor<types...>;
            using base::base;
        };

        template <class... types> struct copy_assign<false, types...> : alias::move_ctor<types...> {
            using base = alias::move_ctor<types...>;
            using storage = alias::storage<types...>;
            using base::base;

            constexpr copy_assign() = default;
            constexpr copy_assign(copy_assign const &) = default;
            constexpr copy_assign(copy_assign &&other) = default;
            constexpr copy_assign &operator=(copy_assign &&) = default;
            constexpr copy_assign &operator=(const copy_assign &rhs) noexcept(traits<types...>::IS_NOTHROW_COPY_ASSIGN) {
                if (!this->is_valid()) {
                    return *this;
                }

                raw_visit(
                        [this, &rhs](auto rhs_index) {
                            if constexpr (rhs_index != variant_npos) {
                                if (this->index_ == rhs_index) {
                                    get<rhs_index>(variant_cast<types...>(*this)) = get<rhs_index>(variant_cast<types...>(rhs));
                                } else {
                                    variant_cast<types...>(*this) =
                                            variant<types...>(in_place_index<rhs_index>, get<rhs_index>(variant_cast<types...>(rhs)));
                                }
                            } else {
                                this->reset();
                            }
                        },
                        variant_cast<types...>(rhs));

                return *this;
            }
        };

        template <class... types> struct copy_ctor<true, types...> : alias::copy_assign<types...> {
            using base = alias::copy_assign<types...>;
            using base::base;
        };

        template <class... types> struct copy_ctor<false, types...> : alias::copy_assign<types...> {
            using base = alias::copy_assign<types...>;
            using base::base;

            constexpr copy_ctor() = default;
            constexpr copy_ctor(copy_ctor const &other) noexcept(traits<types...>::IS_NOTHROW_COPY_CTOR) {
                using type = std::remove_reference_t<decltype(other.value_)>;

                this->index_ = other.index_;
                raw_visit(
                        [this, &other](auto index) {
                            ::new (std::addressof(this->value_)) type(in_place_index<index>, get<index>(variant_cast<types...>(other)));
                        },
                        variant_cast<types...>(other));
            }
            constexpr copy_ctor(copy_ctor &&other) = default;
            constexpr copy_ctor &operator=(copy_ctor &&) = default;
            constexpr copy_ctor &operator=(const copy_ctor &rhs) = default;
        };

        template <class... types> struct variant : alias::copy_ctor<types...> {
            using base = alias::copy_ctor<types...>;
            using base::base;

            constexpr variant() noexcept(traits<types...>::IS_NOTHROW_DEFAULT_CTOR) : variant(in_place_index<0>) {}
        };
    } // namespace base
    struct tag {};

    template <bool> struct default_ctor_enabler {
        constexpr default_ctor_enabler() noexcept = delete;
        constexpr default_ctor_enabler(default_ctor_enabler const &) = default;
        constexpr default_ctor_enabler(default_ctor_enabler &&) = default;
        default_ctor_enabler &operator=(default_ctor_enabler const &) = default;
        default_ctor_enabler &operator=(default_ctor_enabler &&) = default;

        constexpr default_ctor_enabler(tag){};
    };

    template <> struct default_ctor_enabler<true> {
        constexpr default_ctor_enabler() = default;
        constexpr default_ctor_enabler(default_ctor_enabler const &) = default;
        constexpr default_ctor_enabler(default_ctor_enabler &&) = default;
        default_ctor_enabler &operator=(default_ctor_enabler const &) = default;
        default_ctor_enabler &operator=(default_ctor_enabler &&) = default;

        constexpr default_ctor_enabler(tag){};
    };

    template <bool> struct copy_constructor_enabler {
        constexpr copy_constructor_enabler() = default;
        constexpr copy_constructor_enabler(copy_constructor_enabler const &) noexcept = delete;
        constexpr copy_constructor_enabler(copy_constructor_enabler &&) = default;
        copy_constructor_enabler &operator=(copy_constructor_enabler const &) = default;
        copy_constructor_enabler &operator=(copy_constructor_enabler &&) = default;
    };

    template <> struct copy_constructor_enabler<true> {};

    template <bool> struct move_ctor_enabler {
        constexpr move_ctor_enabler() = default;
        constexpr move_ctor_enabler(move_ctor_enabler const &) = default;
        constexpr move_ctor_enabler(move_ctor_enabler &&) noexcept = delete;
        move_ctor_enabler &operator=(move_ctor_enabler const &) = default;
        move_ctor_enabler &operator=(move_ctor_enabler &&) = default;
    };

    template <> struct move_ctor_enabler<true> {};

    template <bool> struct copy_assign_enabler {
        constexpr copy_assign_enabler() = default;
        constexpr copy_assign_enabler(copy_assign_enabler const &) = default;
        constexpr copy_assign_enabler(copy_assign_enabler &&) = default;
        copy_assign_enabler &operator=(copy_assign_enabler const &) noexcept = delete;
        copy_assign_enabler &operator=(copy_assign_enabler &&) = default;
    };

    template <> struct copy_assign_enabler<true> {};

    template <bool> struct move_assign_enabler {
        constexpr move_assign_enabler() = default;
        constexpr move_assign_enabler(move_assign_enabler const &) = default;
        constexpr move_assign_enabler(move_assign_enabler &&) = default;
        move_assign_enabler &operator=(move_assign_enabler const &) noexcept = delete;
        move_assign_enabler &operator=(move_assign_enabler &&) noexcept = delete;
    };

    template <> struct move_assign_enabler<true> {};

    template <typename visitor_type, typename... variant_types>
    constexpr decltype(auto) raw_visit(visitor_type &&visitor, variant_types &&...variants) {
        if ((variants.valueless_by_exception() || ...)) {
            throw bad_variant_access();
        }
        constexpr auto vtable = internal::create_raw_vtable<visitor_type &&, variant_types &&...>();
        return internal::at(vtable, variants.index()...)(std::forward<visitor_type>(visitor),
                                                         std::forward<variant_types>(variants)...);
    }
} // namespace internal

template <std::size_t idx, class variant_type>
constexpr auto get(variant_type &&v) -> decltype(internal::get(in_place_index<idx>, std::forward<variant_type>(v).value_));

template <std::size_t idx, class variant_type>
constexpr auto raw_get(variant_type &&v) -> decltype(internal::get(in_place_index<idx>, std::forward<variant_type>(v).value_));

template <typename type, class variant_type>
constexpr auto get(variant_type &&v)
-> decltype(get<internal::index_of_pack_v<type, internal::extract_type_pack_t<variant_type>>>(std::forward<variant_type>(v)));

template <typename visitor_type, typename... variant_types>
constexpr decltype(auto) visit(visitor_type &&visitor, variant_types &&...variants) {
    if ((variants.valueless_by_exception() || ...)) {
        throw bad_variant_access();
    }
    return internal::raw_visit(
            [&visitor, &variants...](auto... indices) {
                return std::forward<visitor_type>(visitor)(
                        get<indices>(std::forward<variant_types>(variants))...);
            },
            std::forward<variant_types>(variants)...);
}

template <class... types>
class variant : internal::base::variant<types...>,
                internal::default_ctor_enabler<internal::traits<types...>::ENABLE_DEFAULT_CTOR>,
                internal::copy_constructor_enabler<internal::traits<types...>::ENABLE_COPY_CTOR>,
                internal::move_ctor_enabler<internal::traits<types...>::ENABLE_MOVE_CTOR>,
                internal::copy_assign_enabler<internal::traits<types...>::ENABLE_COPY_ASSIGN>,
                internal::move_assign_enabler<internal::traits<types...>::ENABLE_MOVE_ASSIGN> {
    using base = internal::base::variant<types...>;
    using enable_default_ctor = internal::default_ctor_enabler<internal::traits<types...>::ENABLE_DEFAULT_CTOR>;
    template <std::size_t idx, typename = std::enable_if_t<(idx < sizeof...(types))>>
    using ind_to_type = variant_alternative_t<idx, variant>;
    using traits = internal::traits<types...>;

    template <size_t idx, typename variant_type, typename... args_type>
    static void emplace_in_index(variant_type &v, args_type &&...args) {
        auto &&storage = raw_get<idx>(v);
        ::new ((void *)std::addressof(storage))
                std::remove_reference_t<decltype(storage)>(std::forward<args_type>(args)...);
    }

public:
    constexpr variant() = default;

    constexpr variant(const variant &other) = default;

    constexpr variant(variant &&other) = default;

    template <class type, typename = std::enable_if_t<(sizeof...(types) > 0)>,
            typename = std::enable_if_t<!internal::is_inplace_type<type>::value>,
            std::size_t type_j_idx = internal::choose_index<type, types...>, typename type_j = ind_to_type<type_j_idx>,
            typename = std::enable_if_t<internal::is_occur_once<type_j, types...>>,
            typename = std::enable_if_t<std::is_constructible_v<type_j, type>>>
    constexpr variant(type &&t) noexcept(std::is_nothrow_constructible_v<type_j, type>)
            : base(in_place_index<type_j_idx>, std::forward<type>(t)), enable_default_ctor(internal::tag{}) {}

    template <class type, class... arg_types,
            typename = std::enable_if_t<internal::is_occur_once<type, types...> &&
                                        std::is_constructible_v<type, arg_types...>>>
    constexpr explicit variant(in_place_type_t<type>, arg_types &&...args)
            : base(in_place_index<internal::choose_index<type, types...>>, std::forward<arg_types>(args)...),
              enable_default_ctor(internal::tag{}) {}

    template <
            class type, class init_list_type, class... arg_types,
            typename = std::enable_if_t<internal::is_occur_once<type> &&
                                        std::is_constructible_v<type, std::initializer_list<init_list_type> &, arg_types...>>>
    constexpr explicit variant(in_place_type_t<type>, std::initializer_list<init_list_type> il, arg_types &&...args)
            : base(in_place_index<internal::choose_index<type, types...>>, il, std::forward<arg_types>(args)...),
              enable_default_ctor(internal::tag{}) {}

    template <std::size_t idx, class... arg_types, typename = std::enable_if_t<(idx < sizeof...(types))>,
            typename type = ind_to_type<idx>, typename = std::enable_if_t<std::is_constructible_v<type, arg_types...>>>
    constexpr explicit variant(in_place_index_t<idx>, arg_types &&...args)
            : base(in_place_index<idx>, std::forward<arg_types>(args)...), enable_default_ctor(internal::tag{}) {}

    template <
            std::size_t idx, class init_list_type, class... arg_types, typename = std::enable_if_t<(idx < sizeof...(types))>,
            typename t = ind_to_type<idx>,
            typename = std::enable_if_t<std::is_constructible_v<t, std::initializer_list<init_list_type> &, arg_types...>>>
    constexpr explicit variant(in_place_index_t<idx>, std::initializer_list<init_list_type> il, arg_types &&...args)
            : base(in_place_index<idx>, il, std::forward<arg_types>(args)...), enable_default_ctor(internal::tag{}) {}

    constexpr variant &operator=(const variant &rhs) = default;
    constexpr variant &operator=(variant &&rhs) = default;

    template <typename type>
    std::enable_if_t<internal::is_occur_once<ind_to_type<internal::choose_index<type &&, types...>>, types...> &&
                     std::is_constructible_v<ind_to_type<internal::choose_index<type &&, types...>>, type> &&
                     std::is_assignable_v<ind_to_type<internal::choose_index<type &&, types...>> &, type>,
            variant &>
    operator=(type &&rhs) noexcept(
    std::is_nothrow_assignable_v<ind_to_type<internal::choose_index<type &&, types...>> &, type>
    &&std::is_nothrow_constructible_v<ind_to_type<internal::choose_index<type &&, types...>>, type>) {
        constexpr auto index = internal::choose_index<type, types...>;
        if (this->index() == index)
            get<index>(*this) = std::forward<type>(rhs);
        else {
            using type_j = ind_to_type<internal::choose_index<type &&, types...>>;
            if constexpr (std::is_nothrow_constructible_v<type_j, type> || !std::is_nothrow_move_constructible_v<type_j>)
                this->emplace<index>(std::forward<type>(rhs));
            else
                *this = variant(std::forward<type>(rhs));
        }
        return *this;
    }

    template <typename type, typename... Args>
    std::enable_if_t<std::is_constructible_v<type, Args...> && internal::is_occur_once<type, types...>, type &>
    emplace(Args &&...args) {
        constexpr size_t index = internal::index_of_v<type, types...>;
        return this->emplace<index>(std::forward<Args>(args)...);
    }

    template <typename type, typename init_list_type, typename... Args>
    std::enable_if_t<std::is_constructible_v<type, std::initializer_list<init_list_type> &, Args...> &&
                     internal::is_occur_once<type, types...>,
            type &>
    emplace(std::initializer_list<init_list_type> il, Args &&...args) {
        constexpr size_t index = internal::index_of_v<type, types...>;
        return this->emplace<index>(il, std::forward<Args>(args)...);
    }

    template <size_t Np, typename... Args>
    std::enable_if_t<std::is_constructible_v<variant_alternative_t<Np, variant>, Args...>,
            variant_alternative_t<Np, variant> &>
    emplace(Args &&...args) {
        static_assert(Np < sizeof...(types), "the index must be in [0, number of alternatives)");
        using type = variant_alternative_t<Np, variant>;
        if constexpr (std::is_nothrow_constructible_v<type, Args...>) {
            this->reset();
            this->index_ = Np;
            emplace_in_index<Np>(*this, std::forward<Args>(args)...);
        } else if constexpr (std::is_scalar_v<type>) {
            const type tmp(std::forward<Args>(args)...);
            this->reset();
            this->index_ = Np;
            emplace_in_index<Np>(*this, tmp);
        } else {
            this->reset();
            emplace_in_index<Np>(*this, std::forward<Args>(args)...);
            this->index_ = Np;
        }
        return get<Np>(*this);
    }

    void swap(variant &rhs) {
        if ((!this->is_valid() && !rhs.is_valid()) || this == std::addressof(rhs)) {
            return;
        }

        if (this->index_ == rhs.index_) {
            using std::swap;
            internal::raw_visit(
                    [this, &rhs](auto rhs_index) {
                        auto &&rhs_el = get<rhs_index>(rhs);
                        swap(get<rhs_index>(*this), rhs_el);
                    },
                    rhs);
            std::swap(this->index_, rhs.index_);
        } else if (!rhs.is_valid()) {
            rhs = std::move(*this);
            rhs.index_ = this->index_;
            this->reset();
        } else if (!this->is_valid()) {
            *this = std::move(rhs);
            rhs.reset();
        } else {
            auto tmp(std::move(rhs));
            rhs = std::move(*this);
            *this = std::move(tmp);
        }
    }

    [[nodiscard]] constexpr bool valueless_by_exception() const noexcept { return !this->is_valid(); }
    [[nodiscard]] constexpr std::size_t index() const noexcept { return this->index_; }

    template <std::size_t idx, class variant_type>
    friend constexpr auto get(variant_type &&v) -> decltype(internal::get(in_place_index<idx>, std::forward<variant_type>(v).value_));

    template <std::size_t idx, class variant_type>
    friend constexpr auto raw_get(variant_type &&v) -> decltype(internal::get(in_place_index<idx>, std::forward<variant_type>(v).value_));

    template <typename type, class variant_type>
    friend constexpr auto get(variant_type &&v)
    -> decltype(get<internal::index_of_pack_v<type, internal::extract_type_pack_t<variant_type>>>(std::forward<variant_type>(v)));
};

template <typename type, class variant_type>
constexpr auto get(variant_type &&v)
-> decltype(get<internal::index_of_pack_v<type, internal::extract_type_pack_t<variant_type>>>(std::forward<variant_type>(v))) {
    static_assert(internal::is_occur_once_in_pack<type, internal::extract_type_pack_t<variant_type>>,
                  "Type can occur in variant only once");
    return get<internal::index_of_pack_v<type, internal::extract_type_pack_t<variant_type>>>(std::forward<variant_type>(v));
}

template< class... Types >
constexpr bool operator==( const variant<Types...>& v,
                           const variant<Types...>& w ) {
    if (v.index() != w.index()) {
        return false;
    }

    if (v.valueless_by_exception()) {
        return true;
    }

    return internal::raw_visit([&](auto index) {
        return get<index>(v) == get<index>(w);
    }, v);
}

template< class... Types >
constexpr bool operator!=( const variant<Types...>& v,
                           const variant<Types...>& w ) {
    return !(v == w);
}

template< class... Types >
constexpr bool operator<( const variant<Types...>& v,
                          const variant<Types...>& w ) {
    if (w.valueless_by_exception()) {
        return false;
    }

    if (v.valueless_by_exception()) {
        return true;
    }

    if (v.index() < w.index()) {
        return true;
    }

    if (v.index() > w.index()) {
        return false;
    }

    return internal::raw_visit([&](auto index) {
        return get<index>(v) < get<index>(w);
    }, v);
}

template< class... Types >
constexpr bool operator>=( const variant<Types...>& v,
                           const variant<Types...>& w ) {
    return !(v < w);
}

template< class... Types >
constexpr bool operator>( const variant<Types...>& v,
                          const variant<Types...>& w ) {
    return v != w && v >= w;
}

template< class... Types >
constexpr bool operator<=( const variant<Types...>& v,
                           const variant<Types...>& w ) {
    return !(v > w);
}

template <std::size_t idx, class variant_type>
constexpr auto raw_get(variant_type &&v) -> decltype(internal::get(in_place_index<idx>, std::forward<variant_type>(v).value_)) {
    return internal::get(in_place_index<idx>, std::forward<variant_type>(v).value_);
}

template <std::size_t idx, class variant_type>
constexpr auto get(variant_type &&v) -> decltype(internal::get(in_place_index<idx>, std::forward<variant_type>(v).value_)) {
    if (idx != v.index()) {
        throw bad_variant_access("Invalid get");
    }
    return internal::get(in_place_index<idx>, std::forward<variant_type>(v).value_);
}

template <class t, class... types> constexpr bool holds_alternative(const variant<types...> &v) noexcept {
    return internal::index_of_v<t, types...> == v.index();
}

template <size_t index, typename variant_pointer_type>
constexpr decltype(auto) get_if(variant_pointer_type &&ptr) noexcept {
    static_assert(index < internal::extract_type_pack_t<decltype(*ptr)>::size,
                  "Index can't be greater than variant's types list count");
    if (ptr && ptr->index() == index) {
        return std::addressof(get<index>(*ptr));
    } else {
        return static_cast<decltype(std::addressof(get<index>(*ptr)))>(nullptr);
    }
}

template <typename type, class variant_ptr> constexpr decltype(auto) get_if(variant_ptr &&ptr) noexcept {
    static_assert(internal::is_occur_once_in_pack<type, internal::extract_type_pack_t<decltype(*ptr)>>,
                  "Type can occur in variant only once");
    return get_if<internal::index_of_pack_v<type, internal::extract_type_pack_t<decltype(*ptr)>>>(ptr);
}
 