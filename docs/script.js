// 页面加载完成后执行
document.addEventListener('DOMContentLoaded', function() {
    // 隐藏加载动画
    setTimeout(() => {
        document.querySelector('.loader').classList.add('hidden');
    }, 500);
    
    // 导航栏滚动效果
    window.addEventListener('scroll', function() {
        const nav = document.querySelector('nav');
        if (window.scrollY > 50) {
            nav.classList.add('scrolled');
        } else {
            nav.classList.remove('scrolled');
        }
    });
    
    // 汉堡菜单切换
    const burger = document.querySelector('.burger');
    const navLinks = document.querySelector('.nav-links');
    
    burger.addEventListener('click', function() {
        navLinks.classList.toggle('active');
        burger.classList.toggle('toggle');
    });
    
    // 滚动动画
    const animateElements = () => {
        const elements = document.querySelectorAll(
            '.section-title, .about-text, .about-image, .sponsor-card, .timeline-item, .gallery-item, .contact-info'
        );

        const observer = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    entry.target.classList.add('animate');
                } else {
                    entry.target.classList.remove('animate');
                }
            });
        }, { threshold: 0.1 });

        elements.forEach(element => observer.observe(element));
    };

    animateElements();

    const animateOnScroll = function() {
        const elements = document.querySelectorAll('.section-title, .about-text, .about-image, .member-card, .timeline-item, .gallery-item, .contact-info, .contact-form');
        
        elements.forEach(element => {
            const elementPosition = element.getBoundingClientRect().top;
            const screenPosition = window.innerHeight / 1.3;
            
            if (elementPosition < screenPosition) {
                element.classList.add('animate');
            }
        });
    };
    
    // 初始化滚动动画
    window.addEventListener('scroll', animateOnScroll);
    animateOnScroll(); // 初始检查
    
    // 数字动画
    const counters = document.querySelectorAll('.stat-value');
    const speed = 100;
    
    counters.forEach(counter => {
        // 保存原始 data-count 值（因为动画会修改 innerText）
        const originalValue = counter.getAttribute('data-count');
        
        const updateCount = () => {
            const target = originalValue; // 使用保存的原始值
            const currentText = counter.innerText;
            
            // 如果是"7+"这类带符号的值，且当前不是最终值（避免重复动画）
            if (target.includes('+')) {
                const num = parseInt(target);
                
                // 如果当前显示的是数字（而不是"7+"），则继续动画
                if (!currentText.includes('+')) {
                    const count = +currentText || 0;
                    const inc = Math.max(1, Math.ceil(num / speed));
                    
                    if (count < num) {
                        counter.innerText = Math.min(count + inc, num);
                        setTimeout(updateCount, 100);
                    } else {
                        counter.innerText = target; // 显示"7+"
                    }
                }
            } 
            // 普通数字动画
            else {
                const num = +target;
                const count = +currentText || 0;
                const inc = Math.max(1, Math.ceil(num / speed));
                
                if (count < num) {
                    counter.innerText = Math.min(count + inc, num);
                    setTimeout(updateCount, 100);
                } else {
                    counter.innerText = num;
                }
            }
        };
        
        // 观察器配置：每次进入视口时触发动画
        const observer = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    // 重置为初始状态（0或原始值的第一部分）
                    if (originalValue.includes('+')) {
                        counter.innerText = '0'; // 重置为0以便重新增长
                    } else {
                        counter.innerText = '0';
                    }
                    updateCount();
                }
            });
        }, { 
            threshold: 0.1, // 10%进入视口时触发
        });
        
        observer.observe(counter); // 持续观察，不取消
    });
    
    // 平滑滚动
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function(e) {
            e.preventDefault();
            
            const target = document.querySelector(this.getAttribute('href'));
            if (target) {
                window.scrollTo({
                    top: target.offsetTop - 80,
                    behavior: 'smooth'
                });
                
                // 移动端关闭菜单
                if (navLinks.classList.contains('active')) {
                    navLinks.classList.remove('active');
                    burger.classList.remove('toggle');
                }
            }
        });
    });
});