#include "multibotnet/transport/compression.hpp"
#include "multibotnet/utils/logger.hpp"
#include <zlib.h>
#include <lz4.h>
#include <cstring>

namespace multibotnet {

// NoCompressor 实现
bool NoCompressor::compress(const std::vector<uint8_t>& input, 
                           std::vector<uint8_t>& output) {
    output = input;
    return true;
}

bool NoCompressor::decompress(const std::vector<uint8_t>& input,
                             std::vector<uint8_t>& output) {
    output = input;
    return true;
}

// ZlibCompressor 实现
ZlibCompressor::ZlibCompressor(int compression_level)
    : compression_level_(compression_level) {
    if (compression_level_ < 0) compression_level_ = 0;
    if (compression_level_ > 9) compression_level_ = 9;
}

bool ZlibCompressor::compress(const std::vector<uint8_t>& input, 
                             std::vector<uint8_t>& output) {
    if (input.empty()) {
        output.clear();
        return true;
    }
    
    // 估计压缩后大小
    uLongf compressed_size = compressBound(input.size());
    output.resize(compressed_size);
    
    // 执行压缩
    int result = compress2(output.data(), &compressed_size,
                          input.data(), input.size(),
                          compression_level_);
    
    if (result != Z_OK) {
        LOG_ERRORF("ZLIB compression failed: %d", result);
        return false;
    }
    
    // 调整到实际大小
    output.resize(compressed_size);
    return true;
}

bool ZlibCompressor::decompress(const std::vector<uint8_t>& input,
                               std::vector<uint8_t>& output) {
    if (input.empty()) {
        output.clear();
        return true;
    }
    
    // 尝试不同的输出缓冲区大小
    size_t output_size = input.size() * 4;  // 初始假设4倍
    
    for (int attempts = 0; attempts < 5; attempts++) {
        output.resize(output_size);
        uLongf actual_size = output_size;
        
        int result = uncompress(output.data(), &actual_size,
                               input.data(), input.size());
        
        if (result == Z_OK) {
            output.resize(actual_size);
            return true;
        } else if (result == Z_BUF_ERROR) {
            // 缓冲区太小，增加大小
            output_size *= 2;
        } else {
            LOG_ERRORF("ZLIB decompression failed: %d", result);
            return false;
        }
    }
    
    return false;
}

size_t ZlibCompressor::estimateCompressedSize(size_t data_size) const {
    return compressBound(data_size);
}

// Lz4Compressor 实现
Lz4Compressor::Lz4Compressor(bool high_compression)
    : high_compression_(high_compression) {
}

bool Lz4Compressor::compress(const std::vector<uint8_t>& input, 
                            std::vector<uint8_t>& output) {
    if (input.empty()) {
        output.clear();
        return true;
    }
    
    // LZ4需要知道原始大小，所以在开头存储它
    size_t max_compressed_size = LZ4_compressBound(input.size());
    output.resize(sizeof(uint32_t) + max_compressed_size);
    
    // 存储原始大小
    uint32_t original_size = input.size();
    memcpy(output.data(), &original_size, sizeof(uint32_t));
    
    // 执行压缩
    int compressed_size;
    if (high_compression_) {
        compressed_size = LZ4_compress_HC(
            reinterpret_cast<const char*>(input.data()),
            reinterpret_cast<char*>(output.data() + sizeof(uint32_t)),
            input.size(),
            max_compressed_size,
            LZ4HC_CLEVEL_DEFAULT);
    } else {
        compressed_size = LZ4_compress_default(
            reinterpret_cast<const char*>(input.data()),
            reinterpret_cast<char*>(output.data() + sizeof(uint32_t)),
            input.size(),
            max_compressed_size);
    }
    
    if (compressed_size <= 0) {
        LOG_ERROR("LZ4 compression failed");
        return false;
    }
    
    // 调整到实际大小
    output.resize(sizeof(uint32_t) + compressed_size);
    return true;
}

bool Lz4Compressor::decompress(const std::vector<uint8_t>& input,
                              std::vector<uint8_t>& output) {
    if (input.size() < sizeof(uint32_t)) {
        LOG_ERROR("Invalid LZ4 compressed data");
        return false;
    }
    
    // 读取原始大小
    uint32_t original_size;
    memcpy(&original_size, input.data(), sizeof(uint32_t));
    
    // 分配输出缓冲区
    output.resize(original_size);
    
    // 执行解压
    int decompressed_size = LZ4_decompress_safe(
        reinterpret_cast<const char*>(input.data() + sizeof(uint32_t)),
        reinterpret_cast<char*>(output.data()),
        input.size() - sizeof(uint32_t),
        original_size);
    
    if (decompressed_size != static_cast<int>(original_size)) {
        LOG_ERRORF("LZ4 decompression failed: expected %u, got %d", 
                  original_size, decompressed_size);
        return false;
    }
    
    return true;
}

size_t Lz4Compressor::estimateCompressedSize(size_t data_size) const {
    return sizeof(uint32_t) + LZ4_compressBound(data_size);
}

// CompressionManager 实现
CompressionManager& CompressionManager::getInstance() {
    static CompressionManager instance;
    return instance;
}

std::unique_ptr<ICompressor> CompressionManager::createCompressor(CompressionType type) {
    switch (type) {
        case CompressionType::NONE:
            return std::make_unique<NoCompressor>();
        case CompressionType::ZLIB:
            return std::make_unique<ZlibCompressor>();
        case CompressionType::LZ4:
            return std::make_unique<Lz4Compressor>();
        default:
            LOG_WARNF("Unknown compression type: %d, using no compression", 
                     static_cast<int>(type));
            return std::make_unique<NoCompressor>();
    }
}

std::vector<uint8_t> CompressionManager::compressWithHeader(
    const std::vector<uint8_t>& data, CompressionType type) {
    
    auto compressor = createCompressor(type);
    std::vector<uint8_t> compressed_data;
    
    if (!compressor->compress(data, compressed_data)) {
        LOG_ERROR("Compression failed");
        return {};
    }
    
    // 创建带头部的数据
    std::vector<uint8_t> result;
    result.reserve(sizeof(CompressionHeader) + compressed_data.size());
    
    // 填充头部
    CompressionHeader header;
    header.compression_type = static_cast<uint8_t>(type);
    header.uncompressed_size = data.size();
    header.compressed_size = compressed_data.size();
    
    // 写入头部
    result.insert(result.end(), 
                 reinterpret_cast<uint8_t*>(&header),
                 reinterpret_cast<uint8_t*>(&header) + sizeof(header));
    
    // 写入压缩数据
    result.insert(result.end(), compressed_data.begin(), compressed_data.end());
    
    return result;
}

std::vector<uint8_t> CompressionManager::decompressWithHeader(
    const std::vector<uint8_t>& compressed_data) {
    
    if (compressed_data.size() < sizeof(CompressionHeader)) {
        LOG_ERROR("Invalid compressed data: too small");
        return {};
    }
    
    // 读取头部
    CompressionHeader header;
    memcpy(&header, compressed_data.data(), sizeof(header));
    
    // 验证魔术数字
    if (memcmp(header.magic, "MBNC", 4) != 0) {
        LOG_ERROR("Invalid compression header magic");
        return {};
    }
    
    // 验证版本
    if (header.version != 1) {
        LOG_ERRORF("Unsupported compression version: %d", header.version);
        return {};
    }
    
    // 提取压缩数据
    std::vector<uint8_t> compressed_payload(
        compressed_data.begin() + sizeof(header),
        compressed_data.end());
    
    // 创建解压器
    auto decompressor = createCompressor(
        static_cast<CompressionType>(header.compression_type));
    
    // 执行解压
    std::vector<uint8_t> decompressed;
    if (!decompressor->decompress(compressed_payload, decompressed)) {
        LOG_ERROR("Decompression failed");
        return {};
    }
    
    // 验证解压后大小
    if (decompressed.size() != header.uncompressed_size) {
        LOG_ERRORF("Decompression size mismatch: expected %u, got %zu",
                  header.uncompressed_size, decompressed.size());
        return {};
    }
    
    return decompressed;
}

CompressionType CompressionManager::recommendCompression(
    size_t data_size, bool speed_priority) {
    
    // 小数据不压缩
    if (data_size < 1024) {
        return CompressionType::NONE;
    }
    
    // 速度优先选择LZ4
    if (speed_priority) {
        return CompressionType::LZ4;
    }
    
    // 大数据选择ZLIB以获得更好的压缩率
    if (data_size > 100 * 1024) {  // 100KB
        return CompressionType::ZLIB;
    }
    
    // 默认使用LZ4
    return CompressionType::LZ4;
}

} // namespace multibotnet