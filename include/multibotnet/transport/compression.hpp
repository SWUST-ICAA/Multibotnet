#ifndef MULTIBOTNET_TRANSPORT_COMPRESSION_HPP
#define MULTIBOTNET_TRANSPORT_COMPRESSION_HPP

#include <vector>
#include <cstdint>
#include <memory>
#include <string>

namespace multibotnet {

/**
 * @brief 压缩算法枚举
 */
enum class CompressionType {
    NONE = 0,
    ZLIB = 1,
    LZ4 = 2,
    SNAPPY = 3
};

/**
 * @brief 压缩器接口
 */
class ICompressor {
public:
    virtual ~ICompressor() = default;
    
    /**
     * @brief 压缩数据
     * @param input 输入数据
     * @param output 输出缓冲区
     * @return 是否成功
     */
    virtual bool compress(const std::vector<uint8_t>& input, 
                         std::vector<uint8_t>& output) = 0;
    
    /**
     * @brief 解压数据
     * @param input 输入数据
     * @param output 输出缓冲区
     * @return 是否成功
     */
    virtual bool decompress(const std::vector<uint8_t>& input,
                           std::vector<uint8_t>& output) = 0;
    
    /**
     * @brief 获取压缩类型
     * @return 压缩类型
     */
    virtual CompressionType getType() const = 0;
    
    /**
     * @brief 获取压缩比估计
     * @param data_size 数据大小
     * @return 估计的压缩后大小
     */
    virtual size_t estimateCompressedSize(size_t data_size) const = 0;
};

/**
 * @brief 无压缩实现
 */
class NoCompressor : public ICompressor {
public:
    bool compress(const std::vector<uint8_t>& input, 
                 std::vector<uint8_t>& output) override;
    bool decompress(const std::vector<uint8_t>& input,
                   std::vector<uint8_t>& output) override;
    CompressionType getType() const override { return CompressionType::NONE; }
    size_t estimateCompressedSize(size_t data_size) const override { return data_size; }
};

/**
 * @brief ZLIB压缩实现
 */
class ZlibCompressor : public ICompressor {
public:
    ZlibCompressor(int compression_level = 6);
    
    bool compress(const std::vector<uint8_t>& input, 
                 std::vector<uint8_t>& output) override;
    bool decompress(const std::vector<uint8_t>& input,
                   std::vector<uint8_t>& output) override;
    CompressionType getType() const override { return CompressionType::ZLIB; }
    size_t estimateCompressedSize(size_t data_size) const override;
    
private:
    int compression_level_;
};

/**
 * @brief LZ4压缩实现（高速压缩）
 */
class Lz4Compressor : public ICompressor {
public:
    Lz4Compressor(bool high_compression = false);
    
    bool compress(const std::vector<uint8_t>& input, 
                 std::vector<uint8_t>& output) override;
    bool decompress(const std::vector<uint8_t>& input,
                   std::vector<uint8_t>& output) override;
    CompressionType getType() const override { return CompressionType::LZ4; }
    size_t estimateCompressedSize(size_t data_size) const override;
    
private:
    bool high_compression_;
};

/**
 * @brief 压缩管理器
 */
class CompressionManager {
public:
    /**
     * @brief 获取单例实例
     */
    static CompressionManager& getInstance();
    
    /**
     * @brief 创建压缩器
     * @param type 压缩类型
     * @return 压缩器指针
     */
    std::unique_ptr<ICompressor> createCompressor(CompressionType type);
    
    /**
     * @brief 压缩数据（带头部信息）
     * @param data 原始数据
     * @param type 压缩类型
     * @return 压缩后的数据（包含头部）
     */
    std::vector<uint8_t> compressWithHeader(const std::vector<uint8_t>& data,
                                           CompressionType type);
    
    /**
     * @brief 解压数据（自动识别压缩类型）
     * @param compressed_data 压缩数据（包含头部）
     * @return 解压后的数据
     */
    std::vector<uint8_t> decompressWithHeader(const std::vector<uint8_t>& compressed_data);
    
    /**
     * @brief 根据数据大小推荐压缩类型
     * @param data_size 数据大小
     * @param priority 优先级（速度优先或压缩率优先）
     * @return 推荐的压缩类型
     */
    CompressionType recommendCompression(size_t data_size, bool speed_priority = true);
    
private:
    CompressionManager() = default;
    CompressionManager(const CompressionManager&) = delete;
    CompressionManager& operator=(const CompressionManager&) = delete;
    
    // 压缩头部结构
    struct CompressionHeader {
        uint8_t magic[4] = {'M', 'B', 'N', 'C'};  // Magic number
        uint8_t version = 1;
        uint8_t compression_type;
        uint32_t uncompressed_size;
        uint32_t compressed_size;
    } __attribute__((packed));
};

} // namespace multibotnet

#endif // MULTIBOTNET_TRANSPORT_COMPRESSION_HPP