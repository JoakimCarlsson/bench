//! The engine's JSON value tree and strict recursive descent parser, over an
//! allocator. Objects keep members in insertion order; strings, arrays and
//! objects own their storage.
const std = @import("std");

const Allocator = std.mem.Allocator;

/// A JSON value: null, boolean, number, string, array or object.
pub const Value = union(enum) {
    null,
    bool: bool,
    number: f64,
    string: []u8,
    array: std.ArrayList(Value),
    object: std.ArrayList(Member),

    /// An object member: an owned key and its value.
    pub const Member = struct {
        key: []u8,
        value: Value,
    };

    /// Releases everything this value owns.
    pub fn deinit(self: *Value, gpa: Allocator) void {
        switch (self.*) {
            .string => |text| gpa.free(text),
            .array => |*list| {
                for (list.items) |*item| item.deinit(gpa);
                list.deinit(gpa);
            },
            .object => |*list| {
                for (list.items) |*member| {
                    gpa.free(member.key);
                    member.value.deinit(gpa);
                }
                list.deinit(gpa);
            },
            else => {},
        }
        self.* = .null;
    }

    /// The array elements; empty for any other kind.
    pub fn items(self: *const Value) []const Value {
        return switch (self.*) {
            .array => |list| list.items,
            else => &.{},
        };
    }

    /// The object members in insertion order; empty for any other kind.
    pub fn members(self: *const Value) []const Member {
        return switch (self.*) {
            .object => |list| list.items,
            else => &.{},
        };
    }

    /// Whether this value is an array.
    pub fn isArray(self: *const Value) bool {
        return self.* == .array;
    }

    /// The string bytes; empty for any other kind.
    pub fn asString(self: *const Value) []const u8 {
        return switch (self.*) {
            .string => |text| text,
            else => "",
        };
    }

    /// Sets an object member, replacing an existing one and freeing what it
    /// held. The object takes ownership of `key` and `value`.
    fn set(self: *Value, gpa: Allocator, key: []u8, value: Value) Allocator.Error!void {
        const list = &self.object;
        for (list.items) |*member| {
            if (std.mem.eql(u8, member.key, key)) {
                member.value.deinit(gpa);
                member.value = value;
                gpa.free(key);
                return;
            }
        }
        try list.append(gpa, .{ .key = key, .value = value });
    }

    /// Looks up an object member; null when absent or this is not an object.
    pub fn find(self: *const Value, key: []const u8) ?*const Value {
        for (self.members()) |*member| {
            if (std.mem.eql(u8, member.key, key)) return &member.value;
        }
        return null;
    }

    /// Reads a boolean member, or `fallback` when absent or not a boolean.
    pub fn boolOr(self: *const Value, key: []const u8, fallback: bool) bool {
        const found = self.find(key) orelse return fallback;
        return switch (found.*) {
            .bool => |value| value,
            else => fallback,
        };
    }

    /// Reads a number member, or `fallback` when absent or not a number.
    pub fn numberOr(self: *const Value, key: []const u8, fallback: f64) f64 {
        const found = self.find(key) orelse return fallback;
        return switch (found.*) {
            .number => |value| value,
            else => fallback,
        };
    }

    /// Reads a string member without copying it, or an empty string when
    /// absent or not a string.
    pub fn textOr(self: *const Value, key: []const u8) []const u8 {
        const found = self.find(key) orelse return "";
        return found.asString();
    }
};

const ParseError = error{ InvalidJson, OutOfMemory };

const max_depth = 128;

/// Parses JSON text. Fails on malformed input, trailing content or nesting
/// deeper than 128. The caller owns the result.
pub fn parse(gpa: Allocator, source: []const u8) ParseError!Value {
    var parser: Parser = .{ .gpa = gpa, .source = source };
    parser.skipSpace();
    var root = try parser.parseValue(0);
    errdefer root.deinit(gpa);
    parser.skipSpace();
    if (parser.offset != source.len) return error.InvalidJson;
    return root;
}

/// A recursive descent parser over a byte slice.
const Parser = struct {
    gpa: Allocator,
    source: []const u8,
    offset: usize = 0,

    /// Advances past JSON whitespace.
    fn skipSpace(self: *Parser) void {
        while (self.offset < self.source.len) {
            switch (self.source[self.offset]) {
                ' ', '\t', '\n', '\r' => self.offset += 1,
                else => break,
            }
        }
    }

    /// Consumes `character` if it is next.
    fn expect(self: *Parser, character: u8) ParseError!void {
        if (self.offset >= self.source.len or self.source[self.offset] != character) return error.InvalidJson;
        self.offset += 1;
    }

    /// Tests the next character without consuming it.
    fn nextIs(self: *const Parser, character: u8) bool {
        return self.offset < self.source.len and self.source[self.offset] == character;
    }

    /// Consumes a keyword when the input continues with it.
    fn literal(self: *Parser, word: []const u8) ParseError!void {
        if (!std.mem.startsWith(u8, self.source[self.offset..], word)) return error.InvalidJson;
        self.offset += word.len;
    }

    /// Parses any value at the current position.
    fn parseValue(self: *Parser, depth: usize) ParseError!Value {
        if (depth > max_depth or self.offset >= self.source.len) return error.InvalidJson;
        switch (self.source[self.offset]) {
            '{' => return self.parseObject(depth),
            '[' => return self.parseArray(depth),
            '"' => return .{ .string = try self.parseString() },
            't' => {
                try self.literal("true");
                return .{ .bool = true };
            },
            'f' => {
                try self.literal("false");
                return .{ .bool = false };
            },
            'n' => {
                try self.literal("null");
                return .null;
            },
            else => return .{ .number = try self.parseNumber() },
        }
    }

    /// Parses one `"key": value` pair and sets it on `object`.
    fn parseMember(self: *Parser, depth: usize, object: *Value) ParseError!void {
        self.skipSpace();
        const key = try self.parseString();
        errdefer self.gpa.free(key);
        self.skipSpace();
        try self.expect(':');
        self.skipSpace();
        var member = try self.parseValue(depth + 1);
        errdefer member.deinit(self.gpa);
        try object.set(self.gpa, key, member);
    }

    /// Parses one element and appends it to `array`.
    fn parseItem(self: *Parser, depth: usize, array: *Value) ParseError!void {
        self.skipSpace();
        var item = try self.parseValue(depth + 1);
        errdefer item.deinit(self.gpa);
        try array.array.append(self.gpa, item);
    }

    /// Parses an object.
    fn parseObject(self: *Parser, depth: usize) ParseError!Value {
        try self.expect('{');
        var result: Value = .{ .object = .empty };
        errdefer result.deinit(self.gpa);
        self.skipSpace();
        if (self.nextIs('}')) {
            self.offset += 1;
            return result;
        }
        while (true) {
            try self.parseMember(depth, &result);
            self.skipSpace();
            if (self.nextIs(',')) {
                self.offset += 1;
                continue;
            }
            try self.expect('}');
            return result;
        }
    }

    /// Parses an array.
    fn parseArray(self: *Parser, depth: usize) ParseError!Value {
        try self.expect('[');
        var result: Value = .{ .array = .empty };
        errdefer result.deinit(self.gpa);
        self.skipSpace();
        if (self.nextIs(']')) {
            self.offset += 1;
            return result;
        }
        while (true) {
            try self.parseItem(depth, &result);
            self.skipSpace();
            if (self.nextIs(',')) {
                self.offset += 1;
                continue;
            }
            try self.expect(']');
            return result;
        }
    }

    /// Parses a quoted string and decodes its escapes. The caller owns it.
    fn parseString(self: *Parser) ParseError![]u8 {
        try self.expect('"');
        var result: std.ArrayList(u8) = .empty;
        errdefer result.deinit(self.gpa);
        while (true) {
            if (self.offset >= self.source.len) return error.InvalidJson;
            const character = self.source[self.offset];
            self.offset += 1;
            if (character == '"') return result.toOwnedSlice(self.gpa);
            if (character != '\\') {
                try result.append(self.gpa, character);
                continue;
            }
            if (self.offset >= self.source.len) return error.InvalidJson;
            const escape = self.source[self.offset];
            self.offset += 1;
            switch (escape) {
                '"', '\\', '/' => try result.append(self.gpa, escape),
                'b' => try result.append(self.gpa, 0x08),
                'f' => try result.append(self.gpa, 0x0C),
                'n' => try result.append(self.gpa, '\n'),
                'r' => try result.append(self.gpa, '\r'),
                't' => try result.append(self.gpa, '\t'),
                'u' => try self.parseEscapeUnit(&result),
                else => return error.InvalidJson,
            }
        }
    }

    /// Decodes the four hex digits of a unicode escape to UTF-8 and appends
    /// them. Surrogate pairs are not combined.
    fn parseEscapeUnit(self: *Parser, out: *std.ArrayList(u8)) ParseError!void {
        if (self.offset + 4 > self.source.len) return error.InvalidJson;
        var code: u32 = 0;
        for (0..4) |_| {
            const digit = self.source[self.offset];
            self.offset += 1;
            code <<= 4;
            code |= switch (digit) {
                '0'...'9' => digit - '0',
                'a'...'f' => digit - 'a' + 10,
                'A'...'F' => digit - 'A' + 10,
                else => return error.InvalidJson,
            };
        }
        if (code < 0x80) {
            try out.append(self.gpa, @intCast(code));
        } else if (code < 0x800) {
            try out.append(self.gpa, @intCast(0xC0 | (code >> 6)));
            try out.append(self.gpa, @intCast(0x80 | (code & 0x3F)));
        } else {
            try out.append(self.gpa, @intCast(0xE0 | (code >> 12)));
            try out.append(self.gpa, @intCast(0x80 | ((code >> 6) & 0x3F)));
            try out.append(self.gpa, @intCast(0x80 | (code & 0x3F)));
        }
    }

    /// Parses a number token with `std.fmt.parseFloat`, which rounds
    /// correctly.
    fn parseNumber(self: *Parser) ParseError!f64 {
        const start = self.offset;
        if (self.offset < self.source.len and self.source[self.offset] == '-') self.offset += 1;
        while (self.offset < self.source.len) {
            switch (self.source[self.offset]) {
                '0'...'9', '.', 'e', 'E', '+', '-' => self.offset += 1,
                else => break,
            }
        }
        if (self.offset == start) return error.InvalidJson;
        return std.fmt.parseFloat(f64, self.source[start..self.offset]) catch error.InvalidJson;
    }
};
