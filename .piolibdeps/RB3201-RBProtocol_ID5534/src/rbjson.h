#pragma once

#include <unordered_map>
#include <vector>
#include <sstream>

/**
 * \brief JSON-related objects
 */
namespace rbjson {

class Object;

/**
 * \brief Parse a JSON string to an object.
 */
Object *parse(char *buf, size_t size);

/**
 * \brief Base JSON value class, not instanceable.
 */
class Value {
public:
    enum type_t {
        OBJECT,
        ARRAY,
        STRING,
        NUMBER,
        BOOL,
        NIL
    };

    Value(type_t type = NIL);
    virtual ~Value();

    virtual void serialize(std::stringstream& ss) const = 0; //!< Serialize the value to a string
    std::string str() const; //!< Helper that calls serialize() and returns a string

    //!< Get the object type
    type_t getType() const { 
        return m_type;
    }

    //!< Return true if the object is of type NIL
    bool isNil() const {
        return m_type == NIL;
    }

protected:
    type_t m_type;
};

class Array;

/**
 * \brief A JSON Object
 */
class Object : public Value {
public:
    static Object *parse(char *buf, size_t size);

    Object();
    ~Object();

    void serialize(std::stringstream& ss) const;

    bool contains(const char *key) const;

    Value *get(const char *key) const;
    Object *getObject(const char *key) const;
    Array *getArray(const char *key) const;
    std::string getString(const char *key, std::string def = "") const;
    int64_t getInt(const char *key, int64_t def = 0) const;
    double getDouble(const char *key, double def = 0.0) const;
    bool getBool(const char *key, bool def = false) const;

    void set(const char *key, Value *value);
    void set(const char *key, const char *string);
    void set(const char *key, const std::string& str);
    void set(const char *key, int64_t number);

    void remove(const char *key);

private:
    std::unordered_map<std::string, Value*> m_members;
};

/**
 * \brief A JSON Array
 */
class Array : public Value {
public:
    Array();
    ~Array();

    void serialize(std::stringstream& ss) const;

    size_t size() const { return m_items.size(); };

    Value *get(size_t idx) const;
    Object *getObject(size_t idx) const;
    Array *getArray(size_t idx) const;
    std::string getString(size_t idx, std::string def = "") const;
    int64_t getInt(size_t idx, int64_t def = 0) const;
    double getDouble(size_t idx, double def = 0.0) const;
    bool getBool(size_t idx, bool def = false) const;

    void set(size_t idx, Value *value);
    void insert(size_t idx, Value *value);
    void push_back(Value *value) {
        insert(m_items.size(), value);
    }
    void remove(size_t idx);

private:
    std::vector<Value*> m_items;
};

/**
 * \brief A JSON String
 */
class String : public Value {
public:
    explicit String(const char *value = "");
    explicit String(const std::string& value);
    ~String();

    void serialize(std::stringstream& ss) const;

    const std::string& get() const { return m_value; };

private:
    std::string m_value;
};

/**
 * \brief A JSON Number. It's a double internally.
 */
class Number : public Value {
public:
    explicit Number(double value = 0.0);
    ~Number();

    void serialize(std::stringstream& ss) const;

    double get() const { return m_value; };

private:
    double m_value;
};

/**
 * \brief JSON Boolean value
 */
class Bool : public Value {
public:
    explicit Bool(bool value = false);
    ~Bool();

    void serialize(std::stringstream& ss) const;

    bool get() const { return m_value; };

private:
    bool m_value;
};

/**
 * \brief JSON Nil(null) value.
 */
class Nil : public Value {
public:
    void serialize(std::stringstream& ss) const;
};

};
